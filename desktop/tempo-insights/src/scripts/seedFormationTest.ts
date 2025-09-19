// scripts/seedFormationTest.ts 
import { PrismaClient, DeviceState, UserRole } from '@prisma/client';
import bcrypt from 'bcrypt';
import crypto from 'crypto';
import fs from 'fs/promises';
import path from 'path';

const prisma = new PrismaClient();

// Simple log parser to modify GNSS timestamps
function offsetLogTimestamps(logData: Buffer, offsetSeconds: number): Buffer {
  const logString = logData.toString('utf-8');
  const lines = logString.split('\n');
  
  const modifiedLines = lines.map(line => {
    // Look for GNSS time stamps in various formats
    // Format: $GNGGA,145613.00,... (HHMMSS.SS)
    if (line.startsWith('$GNGGA,') || line.startsWith('$GNRMC,')) {
      const parts = line.split(',');
      if (parts.length > 1 && parts[1].length >= 6) {
        const timeStr = parts[1];
        const hours = parseInt(timeStr.substr(0, 2));
        const minutes = parseInt(timeStr.substr(2, 2));
        const seconds = parseFloat(timeStr.substr(4));
        
        // Add offset
        let totalSeconds = hours * 3600 + minutes * 60 + seconds + offsetSeconds;
        
        // Handle day rollover
        if (totalSeconds >= 86400) totalSeconds -= 86400;
        if (totalSeconds < 0) totalSeconds += 86400;
        
        // Convert back to HHMMSS.SS format
        const newHours = Math.floor(totalSeconds / 3600);
        const newMinutes = Math.floor((totalSeconds % 3600) / 60);
        const newSeconds = totalSeconds % 60;
        
        parts[1] = String(newHours).padStart(2, '0') + 
                   String(newMinutes).padStart(2, '0') + 
                   String(newSeconds.toFixed(2)).padStart(5, '0');
        
        return parts.join(',');
      }
    }
    
    // Look for PENV timestamps (different format)
    // NOTE: PENV stamps do not require correction, as they are relative timestamps.
    if (false && line.startsWith('$PENV,')) {
      const parts = line.split(',');
      if (parts.length > 1 && parts[1]) {
        const timestamp = parseInt(parts[1]);
        if (!isNaN(timestamp)) {
          parts[1] = String(timestamp + Math.floor(offsetSeconds * 1000));
          return parts.join(',');
        }
      }
    }
    
    return line;
  });
  
  return Buffer.from(modifiedLines.join('\n'), 'utf-8');
}

async function main() {
  console.log('Creating formation test data...');

  try {
    // Read the sample flight data
    const sampleFlightPath = path.join(__dirname, '../../docs/sample-flight.txt');
    let originalLogData: Buffer;
    
    try {
      originalLogData = await fs.readFile(sampleFlightPath);
    } catch (error) {
      console.error('Could not read sample-flight.txt. Please ensure the file exists at docs/sample-flight.txt');
      throw error;
    }

    // Create John Smith user if doesn't exist
    const johnPassword = await bcrypt.hash('john123', 10);
    const johnSmith = await prisma.user.upsert({
      where: { email: 'john.smith@example.com' },
      update: {},
      create: {
        email: 'john.smith@example.com',
        password: johnPassword,
        name: 'John Smith',
        slug: 'john-smith',
        role: UserRole.USER,
      },
    });
    console.log('Created/found user John Smith:', johnSmith.id);

    // Create device for John Smith if doesn't exist
    const device5 = await prisma.device.upsert({
      where: { bluetoothId: 'C8:43:CA:EB:FE:05' },
      update: {},
      create: {
        bluetoothId: 'C8:43:CA:EB:FE:05',
        name: 'Tempo-BT-0005',
        state: DeviceState.ACTIVE,
        ownerId: johnSmith.id,
        lastSeen: new Date(),
      },
    });
    console.log('Created/found device Tempo-BT-0005:', device5.id);

    // Get admin user
    const adminUser = await prisma.user.findUnique({
      where: { email: 'admin@tempoinsights.local' },
    });
    if (!adminUser) {
      throw new Error('Admin user not found. Run seed script first.');
    }

    // Get admin's first active device
    const adminDevice = await prisma.device.findFirst({
      where: { 
        ownerId: adminUser.id,
        state: DeviceState.ACTIVE
      },
    });
    if (!adminDevice) {
      throw new Error('Admin device not found. Run seed script first.');
    }

    // Create modified log data with 1 second offset
    const modifiedLogData = offsetLogTimestamps(originalLogData, 1.0);

    // Calculate hashes
    const originalHash = crypto
      .createHash('sha256')
      .update(originalLogData)
      .digest('hex');
    
    const modifiedHash = crypto
      .createHash('sha256')
      .update(modifiedLogData)
      .digest('hex');

    // Check if jumps already exist
    const existingAdminJump = await prisma.jumpLog.findUnique({
      where: { hash: originalHash }
    });
    
    const existingJohnJump = await prisma.jumpLog.findUnique({
      where: { hash: modifiedHash }
    });

    let adminJump, johnJump;

    if (!existingAdminJump) {
      // Create jump log for admin user (original data)
      adminJump = await prisma.jumpLog.create({
        data: {
          hash: originalHash,
          rawLog: originalLogData,
          offsets: {},
          flags: { 
            source: 'test-seed',
            originalFile: 'sample-flight.txt' 
          },
          visibleToConnections: true,
          deviceId: adminDevice.id,
          userId: adminUser.id,
          createdAt: new Date(),
        },
      });
      console.log('Created jump log for admin:', adminJump.id);
    } else {
      console.log('Admin jump already exists:', existingAdminJump.id);
      adminJump = existingAdminJump;
    }

    if (!existingJohnJump) {
      // Create jump log for John Smith (modified data with 1 second offset)
      johnJump = await prisma.jumpLog.create({
        data: {
          hash: modifiedHash,
          rawLog: modifiedLogData,
          offsets: {},
          flags: { 
            source: 'test-seed',
            originalFile: 'sample-flight.txt',
            timeOffset: 1.0 
          },
          visibleToConnections: true,
          deviceId: device5.id,
          userId: johnSmith.id,
          createdAt: new Date(new Date().getTime() + 1000), // 1 second later
        },
      });
      console.log('Created jump log for John:', johnJump.id);
    } else {
      console.log('John jump already exists:', existingJohnJump.id);
      johnJump = existingJohnJump;
    }

    // Create device file index entries to prevent re-ingestion
    const adminFileName = `test-formation-admin-${Date.now()}.csv`;
    const johnFileName = `test-formation-john-${Date.now()}.csv`;

    await prisma.deviceFileIndex.upsert({
      where: {
        deviceId_fileName: {
          deviceId: adminDevice.id,
          fileName: adminFileName
        }
      },
      update: {},
      create: {
        deviceId: adminDevice.id,
        fileName: adminFileName,
      },
    });

    await prisma.deviceFileIndex.upsert({
      where: {
        deviceId_fileName: {
          deviceId: device5.id,
          fileName: johnFileName
        }
      },
      update: {},
      create: {
        deviceId: device5.id,
        fileName: johnFileName,
      },
    });

    console.log('\nFormation test data created successfully!');
    console.log('Two jump logs have been created that should be detected as a formation');
    console.log('by the analysis worker due to their matching timestamps.');
    console.log('\nCredentials for John Smith:');
    console.log('Email: john.smith@example.com');
    console.log('Password: john123');
    console.log('\nNext steps:');
    console.log('1. Run the analysis worker to process these jumps');
    console.log('2. Check the FormationSkydive table for the created formation');
    console.log('3. Test the formation review page with the formation ID');

  } catch (error) {
    console.error('Error creating test data:', error);
    throw error;
  } finally {
    await prisma.$disconnect();
  }
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  });