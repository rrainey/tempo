// scripts/test-visibility.ts
// Run with: tsx src/scripts/test-visibility.ts

import { PrismaClient } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { config } from 'dotenv';
import crypto from 'crypto';

config({ path: '.env' });

const prisma = new PrismaClient();

// Initialize Supabase client
const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

async function testVisibilitySettings() {
  console.log('=== Testing Visibility Settings with Supabase Storage ===\n');
  
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
    const visibleContent = Buffer.from('VISIBLE_JUMP');
    const visibleHash = crypto.createHash('sha256').update(visibleContent).digest('hex');
    const visibleId = crypto.randomUUID();
    const visiblePath = `users/${users[0].id}/jumps/${visibleId}/visible.dat`;
    
    // Upload to storage
    await supabase.storage
      .from('jump-logs')
      .upload(visiblePath, visibleContent, {
        contentType: 'application/octet-stream',
      });
    
    const { data: visibleUrl } = supabase.storage
      .from('jump-logs')
      .getPublicUrl(visiblePath);
    
    const visibleJump = await prisma.jumpLog.create({
      data: {
        id: visibleId,
        hash: visibleHash,
        storageUrl: visibleUrl.publicUrl,
        storagePath: visiblePath,
        fileSize: visibleContent.length,
        mimeType: 'application/octet-stream',
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
    const hiddenContent = Buffer.from('HIDDEN_JUMP');
    const hiddenHash = crypto.createHash('sha256').update(hiddenContent).digest('hex');
    const hiddenId = crypto.randomUUID();
    const hiddenPath = `users/${users[1].id}/jumps/${hiddenId}/hidden.dat`;
    
    await supabase.storage
      .from('jump-logs')
      .upload(hiddenPath, hiddenContent, {
        contentType: 'application/octet-stream',
      });
    
    const { data: hiddenUrl } = supabase.storage
      .from('jump-logs')
      .getPublicUrl(hiddenPath);
    
    const hiddenJump = await prisma.jumpLog.create({
      data: {
        id: hiddenId,
        hash: hiddenHash,
        storageUrl: hiddenUrl.publicUrl,
        storagePath: hiddenPath,
        fileSize: hiddenContent.length,
        mimeType: 'application/octet-stream',
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
    const visible2Content = Buffer.from('VISIBLE_JUMP_2');
    const visible2Hash = crypto.createHash('sha256').update(visible2Content).digest('hex');
    const visible2Id = crypto.randomUUID();
    const visible2Path = `users/${users[2].id}/jumps/${visible2Id}/visible2.dat`;
    
    await supabase.storage
      .from('jump-logs')
      .upload(visible2Path, visible2Content, {
        contentType: 'application/octet-stream',
      });
    
    const { data: visible2Url } = supabase.storage
      .from('jump-logs')
      .getPublicUrl(visible2Path);
    
    const visibleJump2 = await prisma.jumpLog.create({
      data: {
        id: visible2Id,
        hash: visible2Hash,
        storageUrl: visible2Url.publicUrl,
        storagePath: visible2Path,
        fileSize: visible2Content.length,
        mimeType: 'application/octet-stream',
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
      console.log(`     Storage: ${jump.storagePath}`);
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
          console.log('      âš ï¸  Formation includes hidden jumps!');
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
      
      console.log(`   âœ" Jump is now ${updated?.visibleToConnections ? 'VISIBLE' : 'HIDDEN'}`);
    }
    
    // 6. Test storage access with visibility
    console.log('\n6. Testing storage access (simulating different users):');
    
    // In a real app, you'd check permissions before allowing download
    // Here we just demonstrate the concept
    for (const { jump, user, visible } of jumps) {
      console.log(`   Jump by ${user.name}: ${visible ? 'VISIBLE' : 'HIDDEN'}`);
      
      // Simulate permission check
      const canAccess = visible || user.id === users[0].id; // User can always see their own
      
      if (canAccess) {
        const { data, error } = await supabase.storage
          .from('jump-logs')
          .download(jump.storagePath!);
        
        if (error) {
          console.log(`     âœ— Download failed: ${error.message}`);
        } else {
          console.log(`     âœ" Can download (${data.size} bytes)`);
        }
      } else {
        console.log(`     ðŸ"' Access denied (hidden jump)`);
      }
    }
    
    // Cleanup
    console.log('\n7. Cleaning up test data...');
    
    // Delete from storage first
    for (const { jump } of jumps) {
      await supabase.storage
        .from('jump-logs')
        .remove([jump.storagePath!]);
    }
    console.log('   âœ" Storage files removed');
    
    // Then delete from database
    for (const { jump } of jumps) {
      await prisma.jumpLog.delete({ where: { id: jump.id } });
    }
    console.log('   âœ" Test jumps removed from database');
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testVisibilitySettings().catch(console.error);