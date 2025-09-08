import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';

// Load environment variables
config({ path: '.env' });

const prisma = new PrismaClient();

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
      // TODO: Implement actual Bluetooth scanning
      // For now, just log the scan cycle

      const startTime = Date.now();

      // Simulate scan time
      await new Promise(resolve => setTimeout(resolve, 1000));

      const duration = Date.now() - startTime;
      console.log(`[BLUETOOTH SCANNER] Scan cycle ${scanId} completed in ${duration}ms`);

      // Update device online/offline status based on lastSeen
      await this.updateDeviceStatuses();

    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error in scan cycle ${scanId}:`, error);
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
