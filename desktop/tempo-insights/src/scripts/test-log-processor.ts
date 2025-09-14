// Save as: src/scripts/test-log-processor.ts
// Run with: tsx src/scripts/test-log-processor.ts

import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';
import crypto from 'crypto';

config({ path: '.env' });

const prisma = new PrismaClient();

async function testLogProcessor() {
  console.log('=== Testing Log Processor Setup ===\n');
  
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
      
      const jumpLog = await prisma.jumpLog.create({
        data: {
          hash: hash,
          rawLog: mockData,
          deviceId: testDevice.id,
          userId: adminUser.id,
          offsets: {},
          flags: {},
          visibleToConnections: true,
          notes: null,
        }
      });
      
      console.log(`   ✓ Created test jump log with ID: ${jumpLog.id}`);
      console.log(`   - Hash: ${hash.substring(0, 16)}...`);
      console.log(`   - Size: ${mockData.length} bytes\n`);
    }
    
    // 3. Show what the processor will see
    console.log('3. Pending logs for processing:');
    
    // For now, all jumps are "pending" since we haven't added analysis fields yet
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
      pendingJumps.forEach((jump, index) => {
        console.log(`   ${index + 1}. Jump ${jump.id}`);
        console.log(`      - Created: ${jump.createdAt.toISOString()}`);
        console.log(`      - User: ${jump.user.name || jump.user.email}`);
        console.log(`      - Device: ${jump.device.name}`);
        console.log(`      - Size: ${jump.rawLog.length} bytes`);
        console.log(`      - Hash: ${jump.hash.substring(0, 16)}...`);
        console.log('');
      });
    }
    
    // 4. Instructions
    console.log('4. Next steps:');
    console.log('   - Run the log processor: tsx workers/log-processor.ts');
    console.log('   - Watch the console output to see processing cycles');
    console.log(`   - The processor will run every 30 seconds`);
    console.log(`   - It will report ${pendingLogs} pending log(s) each cycle`);
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testLogProcessor().catch(console.error);