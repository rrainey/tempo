import { PrismaClient, DeviceState } from '@prisma/client';
import { config } from 'dotenv';
import { BluetoothService } from '../lib/bluetooth/bluetooth.service';

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
  private discoveredDevices = new Map<string, Date>(); // Track discovered devices

  async start() {
    console.log('[BLUETOOTH SCANNER] Starting worker...');
    
    // Check for smpmgr
    const smpmgrAvailable = await this.checkSmpmgr();
    if (!smpmgrAvailable) {
      console.error('[BLUETOOTH SCANNER] smpmgr is not available in PATH');
      console.error('[BLUETOOTH SCANNER] Please install smpmgr and ensure it is in your PATH');
      console.error('[BLUETOOTH SCANNER] Also ensure the plugin path is set correctly (SMPMGR_PLUGIN_PATH env var)');
      process.exit(1);
    }
    
    console.log('[BLUETOOTH SCANNER] smpmgr is available and ready');
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

  private async checkSmpmgr(): Promise<boolean> {
    return bluetooth.checkSmpmgr();
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

  private async discoverDevices(): Promise<any[]> {
    try {
      const devices = await bluetooth.listTempoDevices();
      return devices;
    } catch (error) {
      console.error('[BLUETOOTH SCANNER] Error discovering devices:', error);
      return [];
    }
  }

  private async processDevice(device: any) {
    console.log(`[BLUETOOTH SCANNER] Processing device ${device.name} (${device.bluetoothId})`);
    
    try {
      // Track when we discovered this device
      this.discoveredDevices.set(device.bluetoothId, new Date());

      // Check if device exists in DB
      let dbDevice = await prisma.device.findUnique({
        where: { bluetoothId: device.bluetoothId },
      });

      if (!dbDevice) {
        console.log(`[BLUETOOTH SCANNER] New device found: ${device.name} - creating record`);
        
        // Get the admin user to assign as default owner
        const adminUser = await prisma.user.findFirst({
          where: { role: 'SUPER_ADMIN' },
        });

        if (adminUser) {
          dbDevice = await prisma.device.create({
            data: {
              bluetoothId: device.bluetoothId,
              name: device.name,
              state: DeviceState.PROVISIONING,
              lastSeen: new Date(),
              ownerId: adminUser.id,
            },
          });
          console.log(`[BLUETOOTH SCANNER] Created device record for ${device.name}`);
        } else {
          console.error('[BLUETOOTH SCANNER] No admin user found to assign device');
          return;
        }
      } else {
        // Update existing device
        await prisma.device.update({
          where: { id: dbDevice.id },
          data: {
            lastSeen: new Date(),
            state: dbDevice.state === DeviceState.INACTIVE ? DeviceState.ACTIVE : dbDevice.state,
          },
        });
        console.log(`[BLUETOOTH SCANNER] Updated lastSeen for ${device.name}`);
      }

      // Check for new files
      await this.checkForNewFiles(dbDevice, device.bluetoothId);
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error processing device ${device.bluetoothId}:`, error);
    }
  }

  private async checkForNewFiles(dbDevice: any, bluetoothId: string) {
    try {
      console.log(`[BLUETOOTH SCANNER] Checking for new files on ${dbDevice.name}`);
      
      // Get list of files on device
      const deviceFiles = await bluetooth.listDeviceFiles(bluetoothId);
      
      // Get list of already uploaded files
      const uploadedFiles = await prisma.deviceFileIndex.findMany({
        where: { deviceId: dbDevice.id },
        select: { fileName: true },
      });
      
      const uploadedFileNames = new Set(uploadedFiles.map(f => f.fileName));
      
      // Find new files
      const newFiles = deviceFiles.filter(fileName => {
        // Only consider .dat files as jump logs
        return fileName.endsWith('.dat') && !uploadedFileNames.has(fileName);
      });
      
      if (newFiles.length > 0) {
        console.log(`[BLUETOOTH SCANNER] Found ${newFiles.length} new files on ${dbDevice.name}:`);
        newFiles.forEach(file => console.log(`[BLUETOOTH SCANNER]   - ${file}`));
        
        // Process each new file
        for (const fileName of newFiles) {
          await this.processNewFile(dbDevice, bluetoothId, fileName);
        }
      } else {
        console.log(`[BLUETOOTH SCANNER] No new files on ${dbDevice.name}`);
      }
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error checking files on device ${dbDevice.id}:`, error);
    }
  }

  private async processNewFile(dbDevice: any, bluetoothId: string, fileName: string) {
    try {
      console.log(`[BLUETOOTH SCANNER] Processing new file ${fileName} from ${dbDevice.name}`);
      
      // For now, simulate file processing
      // In Task 59, we'll actually upload the file
      
      // Mark file as uploaded (to prevent re-processing)
      await prisma.deviceFileIndex.create({
        data: {
          deviceId: dbDevice.id,
          fileName: fileName,
        },
      });
      
      console.log(`[BLUETOOTH SCANNER] Marked ${fileName} as processed (stub)`);
      
      // TODO: Actually download and create jump log
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error processing file ${fileName}:`, error);
    }
  }

  private async updateDeviceStatuses() {
    try {
      const cutoffTime = new Date(Date.now() - DISCOVERY_WINDOW * 1000);
      
      // Get all active devices
      const allDevices = await prisma.device.findMany({
        where: {
          state: {
            not: DeviceState.PROVISIONING,
          },
        },
      });

      let updatedCount = 0;

      for (const device of allDevices) {
        const isOnline = device.lastSeen && device.lastSeen > cutoffTime;
        const shouldBeActive = isOnline;
        
        if (shouldBeActive && device.state === DeviceState.INACTIVE) {
          // Mark as active
          await prisma.device.update({
            where: { id: device.id },
            data: { state: DeviceState.ACTIVE },
          });
          updatedCount++;
          console.log(`[BLUETOOTH SCANNER] Marked device ${device.name} as ACTIVE`);
        } else if (!shouldBeActive && device.state === DeviceState.ACTIVE) {
          // Mark as inactive
          await prisma.device.update({
            where: { id: device.id },
            data: { state: DeviceState.INACTIVE },
          });
          updatedCount++;
          console.log(`[BLUETOOTH SCANNER] Marked device ${device.name} as INACTIVE`);
        }
      }

      if (updatedCount > 0) {
        console.log(`[BLUETOOTH SCANNER] Updated ${updatedCount} device statuses`);
      }
    } catch (error) {
      console.error('[BLUETOOTH SCANNER] Error updating device statuses:', error);
    }
  }
}

// Start the scanner
const scanner = new BluetoothScanner();
scanner.start().catch(error => {
  console.error('[BLUETOOTH SCANNER] Fatal error:', error);
  process.exit(1);
});