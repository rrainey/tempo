// Save as: src/scripts/test-file-upload.ts
// Run with: tsx src/scripts/test-file-upload.ts

import { PrismaClient } from '@prisma/client';
import { BluetoothService } from '../lib/bluetooth/bluetooth.service';
import { config } from 'dotenv';
import crypto from 'crypto';
import fs from 'fs/promises';
import path from 'path';

config({ path: '.env.local' });

const prisma = new PrismaClient();
const bluetooth = BluetoothService.getInstance();

// Override the bluetooth service to use sample flight data
async function getSampleFlightData(): Promise<Buffer> {
  try {
    // Read the sample flight data from the docs directory
    const samplePath = path.join(process.cwd(), 'docs', 'sample-flight.txt');
    const fileContent = await fs.readFile(samplePath);
    console.log(`   Loaded sample flight data: ${fileContent.length} bytes`);
    return fileContent;
  } catch (error) {
    console.error('   Error reading sample-flight.txt:', error);
    console.log('   Using fallback mock data');
    // Fallback if file doesn't exist
    return Buffer.from('Mock flight data - sample-flight.txt not found');
  }
}

async function testFileUpload() {
  console.log('=== Testing File Upload to JumpLog ===\n');
  
  try {
    // 1. Get test devices (avoid Tempo-BT-0004)
    console.log('1. Getting test devices:');
    const devices = await prisma.device.findMany({
      where: {
        name: {
          not: 'Tempo-BT-0004', // Reserved for live testing
        },
      },
      include: {
        owner: true,
        lentTo: true,
      },
    });
    
    if (devices.length === 0) {
      console.log('No test devices found. Please run seed script first.');
      return;
    }
    
    for (const device of devices) {
      console.log(`\n   Device: ${device.name} (${device.bluetoothId})`);
      console.log(`   Owner: ${device.owner.name}`);
      if (device.lentTo) {
        console.log(`   Lent to: ${device.lentTo.name}`);
      }
    }
    
    // 2. Simulate file download and upload for first device
    const testDevice = devices[0];
    console.log(`\n2. Testing file upload for ${testDevice.name}:`);
    
    // List files on device
    const files = await bluetooth.listDeviceFiles(testDevice.bluetoothId);
    const jumpFiles = files.filter(f => f.endsWith('.dat'));
    
    console.log(`   Total files: ${files.length}`);
    console.log(`   Jump logs: ${jumpFiles.length}`);
    
    if (jumpFiles.length === 0) {
      console.log('   No jump files to process');
      return;
    }
    
    // Process first jump file
    const testFile = jumpFiles[0];
    console.log(`\n3. Processing ${testFile}:`);
    
    // Instead of downloading from device, use sample flight data
    console.log('   Using sample flight data from docs/sample-flight.txt...');
    const fileContent = await getSampleFlightData();
    
    // Compute hash
    const hash = crypto.createHash('sha256').update(fileContent).digest('hex');
    console.log(`   SHA-256: ${hash}`);
    
    // Check for duplicate
    const existing = await prisma.jumpLog.findUnique({
      where: { hash },
    });
    
    if (existing) {
      console.log('   Jump log already exists (duplicate)');
      console.log(`   Jump ID: ${existing.id}`);
      console.log(`   Created: ${existing.createdAt}`);
    } else {
      // Create new jump log
      const userId = testDevice.lentToId || testDevice.ownerId;
      console.log(`   Creating new jump log for user ${userId}...`);
      
      const jumpLog = await prisma.jumpLog.create({
        data: {
          hash: hash,
          rawLog: fileContent,
          deviceId: testDevice.id,
          userId: userId,
          offsets: {},
          flags: {},
          visibleToConnections: true,
        },
      });
      
      console.log(`   ✓ Created JumpLog with ID: ${jumpLog.id}`);
      
      // Mark file as processed
      await prisma.deviceFileIndex.create({
        data: {
          deviceId: testDevice.id,
          fileName: testFile,
        },
      });
      
      console.log(`   ✓ Marked ${testFile} as processed`);
    }
    
    // 4. Show jump log statistics
    console.log('\n4. Jump log statistics:');
    const stats = await prisma.jumpLog.groupBy({
      by: ['deviceId'],
      _count: true,
    });
    
    for (const stat of stats) {
      const device = devices.find(d => d.id === stat.deviceId);
      if (device) {
        console.log(`   - ${device.name}: ${stat._count} jump(s)`);
      }
    }
    
    // 5. Show recent jump logs
    console.log('\n5. Recent jump logs:');
    const recentJumps = await prisma.jumpLog.findMany({
      take: 5,
      orderBy: { createdAt: 'desc' },
      include: {
        device: true,
        user: true,
      },
    });
    
    for (const jump of recentJumps) {
      console.log(`\n   Jump ${jump.id}:`);
      console.log(`   - Device: ${jump.device.name}`);
      console.log(`   - User: ${jump.user.name}`);
      console.log(`   - Hash: ${jump.hash.substring(0, 16)}...`);
      console.log(`   - Size: ${jump.rawLog.length} bytes`);
      console.log(`   - Created: ${jump.createdAt.toISOString()}`);
      
      // Show first line of data if it's the sample data
      const firstLine = jump.rawLog.toString().split('\n')[0];
      if (firstLine.startsWith('$PVER')) {
        console.log(`   - Data: ${firstLine.substring(0, 40)}...`);
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
testFileUpload().catch(console.error);