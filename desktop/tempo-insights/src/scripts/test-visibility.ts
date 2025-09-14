// Save as: src/scripts/test-visibility.ts
// Run with: tsx src/scripts/test-visibility.ts

import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';
import crypto from 'crypto';

config({ path: '.env' });

const prisma = new PrismaClient();

async function testVisibilitySettings() {
  console.log('=== Testing Visibility Settings ===\n');
  
  try {
    // 1. Create test scenario with visible and hidden jumps
    console.log('1. Creating test jumps with different visibility settings...');
    
    const users = await prisma.user.findMany({ take: 3 });
    if (users.length < 3) {
      console.error('Need at least 3 users for visibility testing.');
      return;
    }
    
    const device = await prisma.device.findFirst();
    if (!device) {
      console.error('No device found.');
      return;
    }
    
    // Base time for consistent exit times
    const baseExitTime = new Date(Date.now() - 10 * 60 * 1000);
    
    // Create jumps with same exit time but different visibility
    const jumps = [];
    
    // User 1: Visible jump
    const visibleJump = await prisma.jumpLog.create({
      data: {
        hash: crypto.createHash('sha256').update(`VISIBLE_${Date.now()}`).digest('hex'),
        rawLog: Buffer.from('VISIBLE_JUMP'),
        deviceId: device.id,
        userId: users[0].id,
        offsets: {},
        flags: {},
        visibleToConnections: true, // VISIBLE
        initialAnalysisTimestamp: new Date(),
        exitTimestampUTC: baseExitTime,
        exitAltitudeFt: 14000,
        exitOffsetSec: 20,
        deploymentOffsetSec: 75,
        freefallTimeSec: 55,
        avgFallRateMph: 120
      }
    });
    jumps.push({ jump: visibleJump, user: users[0], visible: true });
    
    // User 2: Hidden jump (same exit time)
    const hiddenJump = await prisma.jumpLog.create({
      data: {
        hash: crypto.createHash('sha256').update(`HIDDEN_${Date.now()}`).digest('hex'),
        rawLog: Buffer.from('HIDDEN_JUMP'),
        deviceId: device.id,
        userId: users[1].id,
        offsets: {},
        flags: {},
        visibleToConnections: false, // HIDDEN
        initialAnalysisTimestamp: new Date(),
        exitTimestampUTC: new Date(baseExitTime.getTime() + 30000), // 30s later
        exitAltitudeFt: 14000,
        exitOffsetSec: 20,
        deploymentOffsetSec: 75,
        freefallTimeSec: 55,
        avgFallRateMph: 120
      }
    });
    jumps.push({ jump: hiddenJump, user: users[1], visible: false });
    
    // User 3: Visible jump (close exit time)
    const visibleJump2 = await prisma.jumpLog.create({
      data: {
        hash: crypto.createHash('sha256').update(`VISIBLE2_${Date.now()}`).digest('hex'),
        rawLog: Buffer.from('VISIBLE_JUMP_2'),
        deviceId: device.id,
        userId: users[2].id,
        offsets: {},
        flags: {},
        visibleToConnections: true, // VISIBLE
        initialAnalysisTimestamp: new Date(),
        exitTimestampUTC: new Date(baseExitTime.getTime() + 60000), // 60s later
        exitAltitudeFt: 14000,
        exitOffsetSec: 20,
        deploymentOffsetSec: 75,
        freefallTimeSec: 55,
        avgFallRateMph: 120
      }
    });
    jumps.push({ jump: visibleJump2, user: users[2], visible: true });
    
    console.log('   Created jumps:');
    jumps.forEach(({ jump, user, visible }) => {
      console.log(`   - ${user.name || user.email}: ${visible ? 'VISIBLE' : 'HIDDEN'} (Exit: ${jump.exitTimestampUTC?.toISOString()})`);
    });
    
    // 2. Check what the formation detector would see
    console.log('\n2. Checking formation detection query results:');
    
    const visibleJumpsForFormation = await prisma.jumpLog.findMany({
      where: {
        exitTimestampUTC: {
          not: null
        },
        formationParticipant: null,
        visibleToConnections: true // Only visible jumps
      },
      orderBy: {
        exitTimestampUTC: 'desc'
      },
      include: {
        user: true
      }
    });
    
    console.log(`   Formation detector would see ${visibleJumpsForFormation.length} visible jump(s):`);
    visibleJumpsForFormation.forEach(jump => {
      console.log(`   - ${jump.user.name || jump.user.email} at ${jump.exitTimestampUTC?.toISOString()}`);
    });
    
    // 3. Show all jumps regardless of visibility
    console.log('\n3. All jumps (including hidden):');
    
    const allJumps = await prisma.jumpLog.findMany({
      where: {
        exitTimestampUTC: {
          not: null
        },
        formationParticipant: null
      },
      orderBy: {
        exitTimestampUTC: 'desc'
      },
      include: {
        user: true
      }
    });
    
    console.log(`   Total ungrouped jumps: ${allJumps.length}`);
    allJumps.forEach(jump => {
      console.log(`   - ${jump.user.name || jump.user.email}: ${jump.visibleToConnections ? 'VISIBLE' : 'HIDDEN'}`);
    });
    
    // 4. Check existing formations and their participants
    console.log('\n4. Formations and visibility:');
    
    const formations = await prisma.formationSkydive.findMany({
      orderBy: { createdAt: 'desc' },
      take: 5,
      include: {
        participants: {
          include: {
            user: true,
            jumpLog: true
          }
        }
      }
    });
    
    if (formations.length === 0) {
      console.log('   No formations found.');
    } else {
      formations.forEach((formation, index) => {
        console.log(`   ${index + 1}. ${formation.name}`);
        formation.participants.forEach(p => {
          const visibility = p.jumpLog.visibleToConnections ? 'VISIBLE' : 'HIDDEN';
          console.log(`      - ${p.user.name || p.user.email}: ${visibility}`);
        });
        
        // Check if formation has any hidden jumps
        const hasHidden = formation.participants.some(p => !p.jumpLog.visibleToConnections);
        if (hasHidden) {
          console.log('      ⚠️  Formation includes hidden jumps!');
        }
      });
    }
    
    // 5. Test visibility toggle
    console.log('\n5. Testing visibility toggle:');
    
    if (visibleJump) {
      console.log(`   Toggling visibility for ${users[0].name || users[0].email}'s jump...`);
      
      await prisma.jumpLog.update({
        where: { id: visibleJump.id },
        data: { visibleToConnections: false }
      });
      
      const updated = await prisma.jumpLog.findUnique({
        where: { id: visibleJump.id }
      });
      
      console.log(`   ✓ Jump is now ${updated?.visibleToConnections ? 'VISIBLE' : 'HIDDEN'}`);
    }
    
    // Cleanup
    console.log('\n6. Cleaning up test data...');
    for (const { jump } of jumps) {
      await prisma.jumpLog.delete({ where: { id: jump.id } });
    }
    console.log('   ✓ Test jumps removed');
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testVisibilitySettings().catch(console.error);