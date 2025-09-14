// Save as: src/scripts/test-lastseen-updates.ts
// Run with: tsx src/scripts/test-lastseen-updates.ts

import { PrismaClient, DeviceState } from '@prisma/client';
import { config } from 'dotenv';

config({ path: '.env.local' });

const prisma = new PrismaClient();

async function testLastSeenUpdates() {
  console.log('=== Testing LastSeen Updates ===\n');
  
  const DISCOVERY_WINDOW = parseInt(process.env.DISCOVERY_WINDOW || '300');
  console.log(`Discovery window: ${DISCOVERY_WINDOW} seconds (${DISCOVERY_WINDOW / 60} minutes)\n`);
  
  try {
    // 1. Show current device states
    console.log('1. Current device states:');
    const devices = await prisma.device.findMany({
      orderBy: { name: 'asc' },
    });
    
    for (const device of devices) {
      const lastSeenAgo = device.lastSeen 
        ? `${Math.round((Date.now() - device.lastSeen.getTime()) / 1000)}s ago`
        : 'never';
      console.log(`   - ${device.name}: ${device.state}, last seen: ${lastSeenAgo}`);
    }
    
    // 2. Simulate device discovery by updating lastSeen
    console.log('\n2. Simulating device discovery...');
    const deviceToUpdate = devices.find(d => d.state === DeviceState.INACTIVE);
    
    if (deviceToUpdate) {
      console.log(`   Updating ${deviceToUpdate.name} lastSeen to now...`);
      const updated = await prisma.device.update({
        where: { id: deviceToUpdate.id },
        data: { 
          lastSeen: new Date(),
          state: DeviceState.ACTIVE,
        },
      });
      console.log(`   ✓ Updated: ${updated.name} is now ${updated.state}`);
    } else {
      console.log('   No inactive devices to update');
    }
    
    // 3. Check which devices should be marked inactive
    console.log('\n3. Checking for devices that should be INACTIVE:');
    const cutoffTime = new Date(Date.now() - DISCOVERY_WINDOW * 1000);
    console.log(`   Cutoff time: ${cutoffTime.toISOString()}`);
    
    const activeDevices = await prisma.device.findMany({
      where: {
        state: DeviceState.ACTIVE,
      },
    });
    
    for (const device of activeDevices) {
      if (!device.lastSeen || device.lastSeen < cutoffTime) {
        console.log(`   - ${device.name} should be INACTIVE (last seen: ${device.lastSeen?.toISOString() || 'never'})`);
      } else {
        const secondsAgo = Math.round((Date.now() - device.lastSeen.getTime()) / 1000);
        console.log(`   - ${device.name} is correctly ACTIVE (seen ${secondsAgo}s ago)`);
      }
    }
    
    // 4. Show state summary
    console.log('\n4. Device state summary:');
    const summary = await prisma.device.groupBy({
      by: ['state'],
      _count: true,
    });
    
    for (const { state, _count } of summary) {
      console.log(`   - ${state}: ${_count} device(s)`);
    }
    
    // 5. Simulate marking old devices as inactive
    console.log('\n5. Simulating status update based on discovery window:');
    let updateCount = 0;
    
    for (const device of activeDevices) {
      if (!device.lastSeen || device.lastSeen < cutoffTime) {
        await prisma.device.update({
          where: { id: device.id },
          data: { state: DeviceState.INACTIVE },
        });
        console.log(`   ✓ Marked ${device.name} as INACTIVE`);
        updateCount++;
      }
    }
    
    if (updateCount === 0) {
      console.log('   No devices needed status updates');
    }
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testLastSeenUpdates().catch(console.error);