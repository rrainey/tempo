// Save as: src/scripts/test-idempotency.ts
// Run with: tsx src/scripts/test-idempotency.ts

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

async function testIdempotency() {
  console.log('=== Testing Idempotency with Supabase Storage ===\n');
  
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
      console.log(`   âœ" File already indexed (created at ${existingIndex.uploadedAt})`);
    } else {
      // Create file index entry
      const newIndex = await prisma.deviceFileIndex.create({
        data: {
          deviceId: device.id,
          fileName: testFileName,
        },
      });
      console.log(`   âœ" Created file index entry`);
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
      console.log(`   âœ— ERROR: Duplicate was allowed!`);
    } catch (error: any) {
      if (error.code === 'P2002') {
        console.log(`   âœ" Duplicate prevented by unique constraint`);
      } else {
        console.log(`   âœ— Unexpected error: ${error.message}`);
      }
    }
    
    // 3. Test content hash idempotency with Supabase Storage
    console.log(`\n3. Testing content hash idempotency with storage:`);
    console.log(`   Content hash: ${testHash.substring(0, 16)}...`);
    
    // Check if jump log with this hash exists
    const existingJump = await prisma.jumpLog.findUnique({
      where: { hash: testHash },
    });
    
    if (existingJump) {
      console.log(`   âœ" Jump log already exists with this hash`);
      console.log(`   Jump ID: ${existingJump.id}`);
      console.log(`   Created: ${existingJump.createdAt}`);
      console.log(`   Storage: ${existingJump.storagePath}`);
    } else {
      // Create jump log with storage
      const jumpId = crypto.randomUUID();
      const storagePath = `users/${device.ownerId}/jumps/${jumpId}/${testFileName}`;
      
      // Upload to storage
      console.log(`   Uploading to storage: ${storagePath}`);
      const { data: uploadData, error: uploadError } = await supabase.storage
        .from('jump-logs')
        .upload(storagePath, testContent, {
          contentType: 'application/octet-stream',
          upsert: false
        });
      
      if (uploadError) {
        console.error(`   Upload error: ${uploadError.message}`);
        return;
      }
      
      const { data: urlData } = supabase.storage
        .from('jump-logs')
        .getPublicUrl(storagePath);
      
      const newJump = await prisma.jumpLog.create({
        data: {
          id: jumpId,
          hash: testHash,
          storageUrl: urlData.publicUrl,
          storagePath: storagePath,
          fileSize: testContent.length,
          mimeType: 'application/octet-stream',
          deviceId: device.id,
          userId: device.ownerId,
          offsets: {},
          flags: {},
          visibleToConnections: true,
        },
      });
      console.log(`   âœ" Created jump log with ID: ${newJump.id}`);
      console.log(`   âœ" File uploaded to storage`);
    }
    
    // Try to create duplicate jump log - should fail
    console.log(`   Testing duplicate hash prevention...`);
    try {
      const duplicateId = crypto.randomUUID();
      await prisma.jumpLog.create({
        data: {
          id: duplicateId,
          hash: testHash,
          storageUrl: 'dummy-url',
          storagePath: 'dummy-path',
          fileSize: testContent.length,
          mimeType: 'application/octet-stream',
          deviceId: device.id,
          userId: device.ownerId,
          offsets: {},
          flags: {},
          visibleToConnections: true,
        },
      });
      console.log(`   âœ— ERROR: Duplicate hash was allowed!`);
    } catch (error: any) {
      if (error.code === 'P2002') {
        console.log(`   âœ" Duplicate prevented by unique hash constraint`);
      } else {
        console.log(`   âœ— Unexpected error: ${error.message}`);
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
        console.log(`   âœ" Same filename allowed on different device`);
      } catch (error) {
        console.log(`   âœ— Filename blocked on different device (unexpected)`);
      }
      
      // Same content hash should still be blocked globally
      console.log(`   Testing global hash uniqueness...`);
      try {
        const anotherJumpId = crypto.randomUUID();
        await prisma.jumpLog.create({
          data: {
            id: anotherJumpId,
            hash: testHash,
            storageUrl: 'dummy-url-2',
            storagePath: 'dummy-path-2',
            fileSize: testContent.length,
            mimeType: 'application/octet-stream',
            deviceId: device2.id,
            userId: device2.ownerId,
            offsets: {},
            flags: {},
            visibleToConnections: true,
          },
        });
        console.log(`   âœ— ERROR: Same hash allowed from different device!`);
      } catch (error: any) {
        if (error.code === 'P2002') {
          console.log(`   âœ" Hash uniqueness enforced across devices`);
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
    
    // 6. Test storage path uniqueness
    console.log(`\n6. Testing storage path uniqueness:`);
    const existingJumpWithPath = await prisma.jumpLog.findFirst({
      where: { storagePath: { not: null } }
    });
    
    if (existingJumpWithPath) {
      console.log(`   Attempting to use existing path: ${existingJumpWithPath.storagePath}`);
      
      const { error: uploadError } = await supabase.storage
        .from('jump-logs')
        .upload(existingJumpWithPath.storagePath!, Buffer.from('duplicate content'), {
          upsert: false // This should prevent overwriting
        });
      
      if (uploadError) {
        console.log(`   âœ" Storage prevented duplicate path upload: ${uploadError.message}`);
      } else {
        console.log(`   âœ— Storage allowed duplicate path!`);
      }
    }
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testIdempotency().catch(console.error);