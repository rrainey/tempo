import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';
import { BluetoothService } from '../lib/bluetooth/bluetooth.services';


// Load environment variables
config({ path: '.env' });

const prisma = new PrismaClient();
const bluetooth = BluetoothService.getInstance();

// Configuration
const DISCOVERY_WINDOW = parseInt(process.env.DISCOVERY_WINDOW || '300'); // 5 minutes default
const SCAN_INTERVAL = 30000; // 30 seconds between scans

class BluetoothScanner {
  private isRunning = false;
  private scanInterval: NodeJS.Timeout | null = null;

  async start() {
    console.log('[BLUETOOTH SCANNER] Starting worker...');
    console.log(`[BLUETOOTH SCANNER] Discovery window: ${DISCOVERY_WINDOW} seconds`);

    this.isRunning = true;

    // Initial scan
    await this.performScan();

    // Set up interval
    this.scanInterval = setInterval(async () => {
      if (this.isRunning) {
        await this.performScan();
      }
    }, SCAN_INTERVAL);

    // Handle shutdown gracefully
    process.on('SIGINT', () => this.stop());
    process.on('SIGTERM', () => this.stop());
  }

  async stop() {
    console.log('[BLUETOOTH SCANNER] Stopping worker...');
    this.isRunning = false;

    if (this.scanInterval) {
      clearInterval(this.scanInterval);
    }

    await prisma.$disconnect();
    process.exit(0);
  }

  private async performScan() {
    const scanId = Date.now();
    console.log(`[BLUETOOTH SCANNER] Starting scan cycle ${scanId}...`);
    
    try {
      const startTime = Date.now();
      
      // Discover devices
      const devices = await this.discoverDevices();
      
      // Process each discovered device
      for (const device of devices) {
        await this.processDevice(device);
      }
      
      const duration = Date.now() - startTime;
      console.log(`[BLUETOOTH SCANNER] Scan cycle ${scanId} completed in ${duration}ms`);
      console.log(`[BLUETOOTH SCANNER] Processed ${devices.length} devices`);
      
      // Update device online/offline status based on lastSeen
      await this.updateDeviceStatuses();
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error in scan cycle ${scanId}:`, error);
    }
  }

  private async processDevice(device: any) {
    console.log(`[BLUETOOTH SCANNER] Processing device ${device.name} (${device.bluetoothId})`);
    
    try {
      // Check if device exists in DB
      const existingDevice = await prisma.device.findUnique({
        where: { bluetoothId: device.bluetoothId },
      });

      if (!existingDevice) {
        console.log(`[BLUETOOTH SCANNER] New device found: ${device.name}`);
        // TODO: Handle new device discovery
      } else {
        // Update lastSeen
        await prisma.device.update({
          where: { id: existingDevice.id },
          data: {
            lastSeen: new Date(),
            state: 'ACTIVE',
          },
        });
      }

      // TODO: Check for new files and upload
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error processing device ${device.bluetoothId}:`, error);
    }
  }

  private async updateDeviceStatuses() {
    try {
      // Mark devices as offline if not seen within discovery window
      const cutoffTime = new Date(Date.now() - DISCOVERY_WINDOW * 1000);

      const outdatedDevices = await prisma.device.updateMany({
        where: {
          lastSeen: {
            lt: cutoffTime,
          },
          state: 'ACTIVE',
        },
        data: {
          state: 'INACTIVE',
        },
      });

      if (outdatedDevices.count > 0) {
        console.log(`[BLUETOOTH SCANNER] Marked ${outdatedDevices.count} devices as INACTIVE`);
      }
    } catch (error) {
      console.error('[BLUETOOTH SCANNER] Error updating device statuses:', error);
    }
  }

  // Placeholder for device discovery
  private async discoverDevices(): Promise<any[]> {
    // TODO: Implement mcumgr device discovery
    console.log('[BLUETOOTH SCANNER] Discovering Bluetooth devices...');
    return [];
  }

  // Placeholder for file listing
  private async listDeviceFiles(deviceId: string): Promise<string[]> {
    // TODO: Implement mcumgr fs ls
    console.log(`[BLUETOOTH SCANNER] Listing files on device ${deviceId}...`);
    return [];
  }

  // Placeholder for file download
  private async downloadFile(deviceId: string, fileName: string): Promise<Buffer> {
    // TODO: Implement mcumgr fs download
    console.log(`[BLUETOOTH SCANNER] Downloading ${fileName} from device ${deviceId}...`);
    return Buffer.from('');
  }
}

// Start the scanner
const scanner = new BluetoothScanner();
scanner.start().catch(error => {
  console.error('[BLUETOOTH SCANNER] Fatal error:', error);
  process.exit(1);
});
