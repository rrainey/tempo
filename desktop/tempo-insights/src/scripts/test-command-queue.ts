// Save as: src/scripts/test-command-queue.ts
// Run with: tsx src/scripts/test-command-queue.ts

import { PrismaClient, CommandType, CommandStatus } from '@prisma/client';
import { config } from 'dotenv';

config({ path: '.env' });

const prisma = new PrismaClient();

async function testCommandQueue() {
  console.log('=== Testing Device Command Queue ===\n');
  
  try {
    // 1. Get an admin user and a device
    const adminUser = await prisma.user.findFirst({
      where: { role: { in: ['ADMIN', 'SUPER_ADMIN'] } }
    });
    
    if (!adminUser) {
      console.log('No admin user found. Please create one first.');
      return;
    }
    
    const device = await prisma.device.findFirst({
      where: { state: 'ACTIVE' }
    });
    
    if (!device) {
      console.log('No active device found. Please ensure a device is online.');
      return;
    }
    
    console.log(`Using admin user: ${adminUser.name} (${adminUser.id})`);
    console.log(`Using device: ${device.name} (${device.id})\n`);
    
    // 2. Create a test PING command
    console.log('Creating PING command...');
    const pingCommand = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.PING,
        targetDeviceId: device.id,
        sendingUserId: adminUser.id,
        commandData: {}
      }
    });
    console.log(`Created command ${pingCommand.id}\n`);
    
    // 3. Create a BLINK_ON command
    console.log('Creating BLINK_ON command...');
    const blinkCommand = await prisma.deviceCommandQueue.create({
      data: {
        commandType: CommandType.BLINK_ON,
        targetDeviceId: device.id,
        sendingUserId: adminUser.id,
        commandData: { color: 'green' }
      }
    });
    console.log(`Created command ${blinkCommand.id}\n`);
    
    // 4. Wait and check command status
    console.log('Waiting 35 seconds for bluetooth scanner to process commands...');
    await new Promise(resolve => setTimeout(resolve, 35000));
    
    // 5. Check command results
    console.log('\nChecking command results:');
    
    const updatedPing = await prisma.deviceCommandQueue.findUnique({
      where: { id: pingCommand.id }
    });
    
    console.log(`\nPING Command:`);
    console.log(`  Status: ${updatedPing?.commandStatus}`);
    console.log(`  Started: ${updatedPing?.startedAt?.toISOString() || 'Not started'}`);
    console.log(`  Completed: ${updatedPing?.completedAt?.toISOString() || 'Not completed'}`);
    console.log(`  Response: ${JSON.stringify(updatedPing?.responseData)}`);
    console.log(`  Error: ${updatedPing?.errorMessage || 'None'}`);
    
    const updatedBlink = await prisma.deviceCommandQueue.findUnique({
      where: { id: blinkCommand.id }
    });
    
    console.log(`\nBLINK_ON Command:`);
    console.log(`  Status: ${updatedBlink?.commandStatus}`);
    console.log(`  Started: ${updatedBlink?.startedAt?.toISOString() || 'Not started'}`);
    console.log(`  Completed: ${updatedBlink?.completedAt?.toISOString() || 'Not completed'}`);
    console.log(`  Response: ${JSON.stringify(updatedBlink?.responseData)}`);
    console.log(`  Error: ${updatedBlink?.errorMessage || 'None'}`);
    
    // 6. Create a BLINK_OFF command
    if (updatedBlink?.commandStatus === CommandStatus.COMPLETED) {
      console.log('\nCreating BLINK_OFF command to turn off LED...');
      const blinkOffCommand = await prisma.deviceCommandQueue.create({
        data: {
          commandType: CommandType.BLINK_OFF,
          targetDeviceId: device.id,
          sendingUserId: adminUser.id,
          commandData: {}
        }
      });
      console.log(`Created command ${blinkOffCommand.id}`);
      console.log('Command will be processed in next scan cycle (within 30 seconds)');
    }
    
    // 7. List all commands for this device
    console.log('\nAll commands for this device:');
    const allCommands = await prisma.deviceCommandQueue.findMany({
      where: { targetDeviceId: device.id },
      orderBy: { createdAt: 'desc' },
      take: 5
    });
    
    allCommands.forEach(cmd => {
      console.log(`  ${cmd.id.substring(0, 8)}... ${cmd.commandType} - ${cmd.commandStatus}`);
    });
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await prisma.$disconnect();
  }
  
  console.log('\n=== Test Complete ===');
}

// Run the test
testCommandQueue().catch(console.error);