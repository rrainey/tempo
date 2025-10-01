// Save as: src/scripts/test-log-processor.ts
// Run with: tsx src/scripts/test-log-processor.ts

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

async function testLogProcessor() {
  console.log('=== Testing Log Processor Setup with Supabase Storage ===\n');
  
  try {
    // 1. Check current state of jump logs
    console.log('1. Current jump log statistics:');
    
    const totalLogs = await prisma.jumpLog.count();
    const processedLogs = await prisma.jumpLog.count({
      where: {
        initialAnalysisTimestamp: {
          not: null
        }
      }
    });
    const pendingLogs = await prisma.jumpLog.count({
      where: {
        initialAnalysisTimestamp: null
      }
    });
    
    console.log(`   - Total logs: ${totalLogs}`);
    console.log(`   - Processed: ${processedLogs}`);
    console.log(`   - Pending: ${pendingLogs}\n`);
    
    // 2. Create a test jump log if none exist
    if (pendingLogs === 0) {
      console.log('2. Creating test jump log for processing...');
      
      // Get admin user
      const adminUser = await prisma.user.findFirst({
        where: { role: 'SUPER_ADMIN' }
      });
      
      if (!adminUser) {
        console.error('No admin user found. Please run seed first.');
        return;
      }
      
      // Get or create a test device
      let testDevice = await prisma.device.findFirst({
        where: { name: 'Test-Device-Processor' }
      });
      
      if (!testDevice) {
        testDevice = await prisma.device.create({
          data: {
            bluetoothId: 'FF:EE:DD:CC:BB:AA',
            name: 'Test-Device-Processor',
            state: 'ACTIVE',
            ownerId: adminUser.id,
          }
        });
      }
      
      // Create test jump log with mock data
      const mockData = Buffer.from('MOCK_JUMP_DATA_FOR_TESTING');
      const hash = crypto.createHash('sha256').update(mockData).digest('hex');
      const jumpId = crypto.randomUUID();
      const storagePath = `users/${adminUser.id}/jumps/${jumpId}/test-jump.dat`;
      
      // Upload to Supabase Storage
      console.log(`   Uploading mock data to storage...`);
      const { data: uploadData, error: uploadError } = await supabase.storage
        .from('jump-logs')
        .upload(storagePath, mockData, {
          contentType: 'application/octet-stream',
          upsert: false
        });
      
      if (uploadError) {
        console.error(`   Upload failed: ${uploadError.message}`);
        return;
      }
      
      const { data: urlData } = supabase.storage
        .from('jump-logs')
        .getPublicUrl(storagePath);
      
      const jumpLog = await prisma.jumpLog.create({
        data: {
          id: jumpId,
          hash: hash,
          storageUrl: urlData.publicUrl,
          storagePath: storagePath,
          fileSize: mockData.length,
          mimeType: 'application/octet-stream',
          deviceId: testDevice.id,
          userId: adminUser.id,
          offsets: {},
          flags: {},
          visibleToConnections: true,
          notes: null,
        }
      });
      
      console.log(`   âœ" Created test jump log with ID: ${jumpLog.id}`);
      console.log(`   - Hash: ${hash.substring(0, 16)}...`);
      console.log(`   - Size: ${mockData.length} bytes`);
      console.log(`   - Storage: ${storagePath}\n`);
    }
    
    // 3. Show what the processor will see
    console.log('3. Pending logs for processing:');
    
    const pendingJumps = await prisma.jumpLog.findMany({
      where: {
        initialAnalysisTimestamp: null
      },
      orderBy: {
        createdAt: 'desc'
      },
      take: 5,
      include: {
        user: {
          select: {
            name: true,
            email: true
          }
        },
        device: {
          select: {
            name: true
          }
        }
      }
    });
    
    if (pendingJumps.length === 0) {
      console.log('   No pending jumps found\n');
    } else {
      for (const [index, jump] of pendingJumps.entries()) {
        console.log(`   ${index + 1}. Jump ${jump.id}`);
        console.log(`      - Created: ${jump.createdAt.toISOString()}`);
        console.log(`      - User: ${jump.user.name || jump.user.email}`);
        console.log(`      - Device: ${jump.device.name}`);
        console.log(`      - Size: ${jump.fileSize} bytes`);
        console.log(`      - Hash: ${jump.hash.substring(0, 16)}...`);
        console.log(`      - Storage: ${jump.storagePath}`);
        
        // Test that we can download the file
        if (jump.storagePath) {
          const { data, error } = await supabase.storage
            .from('jump-logs')
            .download(jump.storagePath);
          
          if (error) {
            console.log(`      - Storage check: âœ— ${error.message}`);
          } else {
            console.log(`      - Storage check: âœ" File accessible (${data.size} bytes)`);
          }
        }
        
        console.log('');
      }
    }
    
    // 4. Simulate what the processor would do
    console.log('4. Simulating log processor workflow:');
    
    if (pendingJumps.length > 0) {
      const sampleJump = pendingJumps[0];
      console.log(`   Processing jump ${sampleJump.id}...`);
      
      // Download file from storage
      console.log('   - Downloading from storage...');
      const { data: fileData, error: downloadError } = await supabase.storage
        .from('jump-logs')
        .download(sampleJump.storagePath!);
      
      if (downloadError) {
        console.log(`   âœ— Download failed: ${downloadError.message}`);
      } else {
        const fileBuffer = Buffer.from(await fileData.arrayBuffer());
        console.log(`   âœ" Downloaded ${fileBuffer.length} bytes`);
        
        // Verify hash
        const downloadedHash = crypto.createHash('sha256')
          .update(fileBuffer)
          .digest('hex');
        
        if (downloadedHash === sampleJump.hash) {
          console.log('   âœ" Hash verification passed');
        } else {
          console.log('   âœ— Hash mismatch!');
        }
        
        console.log('   - Would parse log data here...');
        console.log('   - Would detect exit/deployment/landing...');
        console.log('   - Would calculate metrics...');
        console.log('   - Would update database with results');
      }
    }
    
    // 5. Instructions
    console.log('\n5. Next steps:');
    console.log('   - Run the log processor: tsx workers/log-processor.ts');
    console.log('   - The processor will:');
    console.log('     1. Query for pending logs (initialAnalysisTimestamp = null)');
    console.log('     2. Download each log file from Supabase Storage');
    console.log('     3. Parse and analyze the log data');
    console.log('     4. Update the database with analysis results');
    console.log('     5. Check for formation grouping opportunities');
    console.log(`   - Currently ${pendingLogs} log(s) are waiting for analysis`);
    
    // 6. Storage bucket info
    console.log('\n6. Storage bucket status:');
    
    // List some files in the bucket to verify it's working
    const { data: files, error: listError } = await supabase.storage
      .from('jump-logs')
      .list('users', {
        limit: 5,
        offset: 0
      });
    
    if (listError) {
      console.log(`   âœ— Could not list bucket contents: ${listError.message}`);
    } else {
      console.log(`   âœ" Bucket 'jump-logs' is accessible`);
      if (files && files.length > 0) {
        console.log(`   Found ${files.length} user folder(s) in bucket`);
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
testLogProcessor().catch(console.error);