// Save as: src/scripts/test-idempotency.ts
// Run with: tsx src/scripts/test-idempotency.ts

import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';
import crypto from 'crypto';

config({ path: '.env' });

const prisma = new PrismaClient();

async function testIdempotency() {
  console.log('=== Testing Idempotency ===\n');
  
  try {
    // 1. Get a test device
    console.log('1. Getting test device:');
    const device = await prisma.device.findFirst({
      where: {
        name: {
          not: 'Tempo-BT-0004', // Avoid live device
        },
      },
    });
    
    if (!device) {
      console.log('No test device found. Please run seed script first.');
      return;
    }
    
    console.log(`   Using device: ${device.name}`);
    
    // 2. Create a test file entry
    const testFileName = '20250120/TEST1234/flight.dat';
    const testContent = Buffer.from('Test flight data content');
    const testHash = crypto.createHash('sha256').update(testContent).digest('hex');
    
    console.log(`\n2. Testing filename-based idempotency:`);
    console.log(`   Test file: ${testFileName}`);
    
    // Check if file already indexed
    const existingIndex = await prisma.deviceFileIndex.findFirst({
      where: {
        deviceId: device.id,
        fileName: testFileName,
      },
    });
    
    if (existingIndex) {
      console.log(`   ✓ File already indexed (created at ${existingIndex.uploadedAt})`);
    } else {
      // Create file index entry
      const newIndex = await prisma.deviceFileIndex.create({
        data: {
          deviceId: device.id,
          fileName: testFileName,
        },
      });
      console.log(`   ✓ Created file index entry`);
    }
    
    // Try to create duplicate - should fail
    console.log(`   Testing duplicate prevention...`);
    try {
      await prisma.deviceFileIndex.create({
        data: {
          deviceId: device.id,
          fileName: testFileName,
        },
      });
      console.log(`   ✗ ERROR: Duplicate was allowed!`);
    } catch (error: any) {
      if (error.code === 'P2002') {
        console.log(`   ✓ Duplicate prevented by unique constraint`);
      } else {
        console.log(`   ✗ Unexpected error: ${error.message}`);
      }
    }
    
    // 3. Test content hash idempotency
    console.log(`\n3. Testing content hash idempotency:`);
    console.log(`   Content hash: ${testHash.substring(0, 16)}...`);
    
    // Check if jump log with this hash exists
    const existingJump = await prisma.jumpLog.findUnique({
      where: { hash: testHash },
    });
    
    if (existingJump) {
      console.log(`   ✓ Jump log already exists with this hash`);
      console.log(`   Jump ID: ${existingJump.id}`);
      console.log(`   Created: ${existingJump.createdAt}`);
    } else {
      // Create jump log
      const newJump = await prisma.jumpLog.create({
        data: {
          hash: testHash,
          rawLog: testContent,
          deviceId: device.id,
          userId: device.ownerId,
          offsets: {},
          flags: {},
          visibleToConnections: true,
        },
      });
      console.log(`   ✓ Created jump log with ID: ${newJump.id}`);
    }
    
    // Try to create duplicate jump log - should fail
    console.log(`   Testing duplicate hash prevention...`);
    try {
      await prisma.jumpLog.create({
        data: {
          hash: testHash,
          rawLog: testContent,
          deviceId: device.id,
          userId: device.ownerId,
          offsets: {},
          flags: {},
          visibleToConnections: true,
        },
      });
      console.log(`   ✗ ERROR: Duplicate hash was allowed!`);
    } catch (error: any) {
      if (error.code === 'P2002') {
        console.log(`   ✓ Duplicate prevented by unique hash constraint`);
      } else {
        console.log(`   ✗ Unexpected error: ${error.message}`);
      }
    }
    
    // 4. Test cross-device idempotency
    console.log(`\n4. Testing cross-device idempotency:`);
    
    // Get another device
    const device2 = await prisma.device.findFirst({
      where: {
        id: { not: device.id },
        name: { not: 'Tempo-BT-0004' },
      },
    });
    
    if (device2) {
      console.log(`   Second device: ${device2.name}`);
      
      // Same filename on different device should be allowed
      try {
        await prisma.deviceFileIndex.create({
          data: {
            deviceId: device2.id,
            fileName: testFileName,
          },
        });
        console.log(`   ✓ Same filename allowed on different device`);
      } catch (error) {
        console.log(`   ✗ Filename blocked on different device (unexpected)`);
      }
      
      // Same content hash should still be blocked globally
      console.log(`   Testing global hash uniqueness...`);
      try {
        await prisma.jumpLog.create({
          data: {
            hash: testHash,
            rawLog: testContent,
            deviceId: device2.id,
            userId: device2.ownerId,
            offsets: {},
            flags: {},
            visibleToConnections: true,
          },
        });
        console.log(`   ✗ ERROR: Same hash allowed from different device!`);
      } catch (error: any) {
        if (error.code === 'P2002') {
          console.log(`   ✓ Hash uniqueness enforced across devices`);
        }
      }
    } else {
      console.log(`   No second device available for cross-device test`);
    }
    
    // 5. Show idempotency statistics
    console.log(`\n5. Idempotency statistics:`);
    
    // Count file indices per device
    const fileStats = await prisma.deviceFileIndex.groupBy({
      by: ['deviceId'],
      _count: true,
    });
    
    console.log(`   File indices by device:`);
    for (const stat of fileStats) {
      const statDevice = await prisma.device.findUnique({
        where: { id: stat.deviceId },
      });
      if (statDevice) {
        console.log(`   - ${statDevice.name}: ${stat._count} files indexed`);
      }
    }
    
    // Count unique jump logs
    const jumpCount = await prisma.jumpLog.count();
    console.log(`\n   Total unique jump logs: ${jumpCount}`);
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testIdempotency().catch(console.error);