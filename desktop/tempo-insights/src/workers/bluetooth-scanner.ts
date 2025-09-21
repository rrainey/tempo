import { PrismaClient, DeviceState } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { config } from 'dotenv';
import { BluetoothService } from '../lib/bluetooth/bluetooth.service';
import crypto from 'crypto';

// Load environment variables
config({ path: '.env' });

const prisma = new PrismaClient();
const bluetooth = BluetoothService.getInstance();
const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

// Configuration
const DISCOVERY_WINDOW = parseInt(process.env.DISCOVERY_WINDOW || '300'); // 5 minutes default
const SCAN_INTERVAL = 30000; // 30 seconds between scans
const INTERNAL_API_TOKEN = process.env.INTERNAL_API_TOKEN || 'default-internal-token';

// Track file processing state
interface FileProcessingState {
  deviceId: string;
  lastCheckTime: Date;
  knownFiles: Set<string>;
}

class BluetoothScanner {
  private isRunning = false;
  private scanInterval: NodeJS.Timeout | null = null;
  private discoveredDevices = new Map<string, Date>(); // Track all discovered devices by name
  private fileProcessingState = new Map<string, FileProcessingState>(); // Track file state per device

  async start() {
    console.log('[BLUETOOTH SCANNER] Starting worker...');
    console.log(`[BLUETOOTH SCANNER] Configuration:`);
    console.log(`[BLUETOOTH SCANNER]   - Discovery window: ${DISCOVERY_WINDOW} seconds`);
    console.log(`[BLUETOOTH SCANNER]   - Scan interval: ${SCAN_INTERVAL / 1000} seconds`);
    
    // Check for smpmgr
    const smpmgrAvailable = await this.checkSmpmgr();
    if (!smpmgrAvailable) {
      console.error('[BLUETOOTH SCANNER] smpmgr is not available in PATH');
      console.error('[BLUETOOTH SCANNER] Please install smpmgr and ensure it is in your PATH');
      console.error('[BLUETOOTH SCANNER] Also ensure the plugin path is set correctly (SMPMGR_PLUGIN_PATH env var)');
      process.exit(1);
    }
    
    console.log('[BLUETOOTH SCANNER] smpmgr is available and ready');
    
    // Initialize file processing state from database
    await this.initializeFileProcessingState();
    
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

  private async initializeFileProcessingState() {
    console.log('[BLUETOOTH SCANNER] Initializing file processing state...');
    
    // Load all known devices and their uploaded files
    const devices = await prisma.device.findMany({
      include: {
        fileIndex: {
          select: { fileName: true },
        },
      },
    });

    for (const device of devices) {
      const knownFiles = new Set(device.fileIndex.map(f => f.fileName));
      this.fileProcessingState.set(device.id, {
        deviceId: device.id,
        lastCheckTime: new Date(),
        knownFiles,
      });
      
      if (knownFiles.size > 0) {
        console.log(`[BLUETOOTH SCANNER] Device ${device.name} has ${knownFiles.size} known files`);
      }
    }
  }

  private async performScan() {
    const scanId = Date.now();
    console.log(`[BLUETOOTH SCANNER] Starting scan cycle ${scanId}...`);
    
    try {
      const startTime = Date.now();
      
      // Track devices seen in this scan
      const currentScanDevices = new Set<string>();
      
      // Discover devices
      const devices = await this.discoverDevices();
      
      // Process each discovered device
      for (const device of devices) {
        currentScanDevices.add(device.name);
        await this.processDevice(device);
      }
      
      const duration = Date.now() - startTime;
      console.log(`[BLUETOOTH SCANNER] Scan cycle ${scanId} completed in ${duration}ms`);
      console.log(`[BLUETOOTH SCANNER] Processed ${devices.length} devices`);
      console.log(`[BLUETOOTH SCANNER] Total devices tracked this session: ${this.discoveredDevices.size}`);
      
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
      const now = new Date();
      
      // Track when we discovered this device
      this.discoveredDevices.set(device.name, now);

      // Check if device exists in DB by name
      let dbDevice = await prisma.device.findFirst({
        where: { name: device.name },
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
              lastSeen: now,
              ownerId: adminUser.id,
            },
          });
          console.log(`[BLUETOOTH SCANNER] Created device record for ${device.name}`);
          
          // Initialize file processing state for new device
          this.fileProcessingState.set(dbDevice.id, {
            deviceId: dbDevice.id,
            lastCheckTime: now,
            knownFiles: new Set(),
          });
        } else {
          console.error('[BLUETOOTH SCANNER] No admin user found to assign device');
          return;
        }
      } else {
        // Update existing device - including bluetoothId in case it changed
        const wasInactive = dbDevice.state === DeviceState.INACTIVE;
        
        await prisma.device.update({
          where: { id: dbDevice.id },
          data: {
            bluetoothId: device.bluetoothId,
            lastSeen: now,
            // Only transition from INACTIVE to ACTIVE, don't change PROVISIONING state
            state: wasInactive ? DeviceState.ACTIVE : dbDevice.state,
          },
        });
        
        if (wasInactive) {
          console.log(`[BLUETOOTH SCANNER] Device ${device.name} is back online (INACTIVE -> ACTIVE)`);
        } else {
          console.log(`[BLUETOOTH SCANNER] Updated lastSeen for ${device.name}`);
        }
      }

      // Check for new files
      await this.checkForNewFiles(dbDevice, device.name);
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error processing device ${device.name}:`, error);
    }
  }

  private async checkForNewFiles(dbDevice: any, deviceName: string) {
    try {
      console.log(`[BLUETOOTH SCANNER] Checking for new files on ${deviceName}`);
      
      // Get current file list from device
      const deviceFiles = await bluetooth.listDeviceFiles(deviceName);
      console.log(`[BLUETOOTH SCANNER] Device ${deviceName} reports ${deviceFiles.length} total files`);
      
      // Get or create file processing state
      let fileState = this.fileProcessingState.get(dbDevice.id);
      if (!fileState) {
        // Load from database if not in memory
        const uploadedFiles = await prisma.deviceFileIndex.findMany({
          where: { deviceId: dbDevice.id },
          select: { fileName: true },
        });
        
        fileState = {
          deviceId: dbDevice.id,
          lastCheckTime: new Date(),
          knownFiles: new Set(uploadedFiles.map(f => f.fileName)),
        };
        this.fileProcessingState.set(dbDevice.id, fileState);
      }
      
      // Find new files (only .dat files are jump logs)
      const jumpLogFiles = deviceFiles.filter(fileName => fileName.endsWith('.dat'));
      const newFiles = jumpLogFiles.filter(fileName => !fileState.knownFiles.has(fileName));
      
      if (newFiles.length > 0) {
        console.log(`[BLUETOOTH SCANNER] Found ${newFiles.length} new jump log(s) on ${deviceName}:`);
        newFiles.forEach((file, index) => {
          console.log(`[BLUETOOTH SCANNER]   ${index + 1}. ${file}`);
        });
        
        // Process each new file
        for (const fileName of newFiles) {
          const success = await this.processNewFile(dbDevice, deviceName, fileName);
          if (success) {
            // Add to known files in memory
            fileState.knownFiles.add(fileName);
          }
        }
        
        // Update last check time
        fileState.lastCheckTime = new Date();
      } else {
        console.log(`[BLUETOOTH SCANNER] No new jump logs on ${deviceName}`);
        console.log(`[BLUETOOTH SCANNER]   - Total files: ${deviceFiles.length}`);
        console.log(`[BLUETOOTH SCANNER]   - Jump logs (.dat): ${jumpLogFiles.length}`);
        console.log(`[BLUETOOTH SCANNER]   - Already processed: ${fileState.knownFiles.size}`);
      }
      
    } catch (error) {
      console.error(`[BLUETOOTH SCANNER] Error checking files on device ${dbDevice.id}:`, error);
    }
  }

private async processNewFile(dbDevice: any, deviceName: string, fileName: string): Promise<boolean> {
  try {
    console.log(`[BLUETOOTH SCANNER] Processing new file ${fileName} from ${deviceName}`);
    
    // Download the file using smpmgr
    console.log(`[BLUETOOTH SCANNER] Downloading ${fileName}...`);
    const fileContent = await bluetooth.downloadFile(deviceName, fileName);
    
    // Verify we got actual content
    if (!fileContent || fileContent.length === 0) {
      console.error(`[BLUETOOTH SCANNER] Downloaded file is empty`);
      return false;
    }
    
    // Compute SHA-256 hash of the file content
    const hash = crypto.createHash('sha256').update(fileContent).digest('hex');
    console.log(`[BLUETOOTH SCANNER] File hash: ${hash.substring(0, 16)}...`);
    
    // Check if we already have this jump log (by hash)
    const existingJumpLog = await prisma.jumpLog.findUnique({
      where: { hash },
    });
    
    if (existingJumpLog) {
      console.log(`[BLUETOOTH SCANNER] Jump log already exists with hash ${hash.substring(0, 16)}... (duplicate file)`);
      console.log(`[BLUETOOTH SCANNER]   - Existing Jump ID: ${existingJumpLog.id}`);
      console.log(`[BLUETOOTH SCANNER]   - Created: ${existingJumpLog.createdAt.toISOString()}`);
      console.log(`[BLUETOOTH SCANNER]   - Device: ${existingJumpLog.deviceId}`);
      
      // Still mark the file as processed to avoid re-downloading
      try {
        await prisma.deviceFileIndex.create({
          data: {
            deviceId: dbDevice.id,
            fileName: fileName,
          },
        });
        console.log(`[BLUETOOTH SCANNER]   - Marked ${fileName} as processed for this device`);
      } catch (indexError: any) {
        if (indexError.code === 'P2002') {
          console.log(`[BLUETOOTH SCANNER]   - File already marked as processed for this device`);
        } else {
          throw indexError;
        }
      }
      
      return true;
    }
    
    // Determine the user ID for this jump log
    let userId = dbDevice.ownerId;
    
    // If device is lent out, assign to the borrower
    if (dbDevice.lentToId) {
      userId = dbDevice.lentToId;
      console.log(`[BLUETOOTH SCANNER] Device is lent out, assigning jump to borrower`);
    }
    
    // Upload to Supabase Storage
    const storagePath = `${userId}/${dbDevice.id}/${hash}.log`;
    console.log(`[BLUETOOTH SCANNER] Uploading to Supabase Storage: ${storagePath}`);
    
    const { data: uploadData, error: uploadError } = await supabase.storage
      .from('jump-logs')
      .upload(storagePath, fileContent, {
        contentType: 'application/octet-stream',
        upsert: false
      });
    
    if (uploadError) {
      throw new Error(`Failed to upload to Supabase Storage: ${uploadError.message}`);
    }
    
    console.log(`[BLUETOOTH SCANNER] Successfully uploaded to storage`);
    
    // Get the storage URL
    const storageUrl = `${process.env.NEXT_PUBLIC_SUPABASE_URL}/storage/v1/object/jump-logs/${storagePath}`;
    
    // Create the jump log entry (without rawLog field)
    const jumpLog = await prisma.jumpLog.create({
      data: {
        hash: hash,
        storageUrl: storageUrl,
        storagePath: storagePath,
        fileSize: fileContent.length,
        mimeType: 'application/octet-stream',
        deviceId: dbDevice.id,
        userId: userId,
        offsets: {}, // Will be populated by analysis worker
        flags: {
          originalFileName: fileName,
          uploadedAt: new Date().toISOString()
        },
        visibleToConnections: true,
        notes: null,
      },
    });
    
    console.log(`[BLUETOOTH SCANNER] Created JumpLog with ID ${jumpLog.id} for user ${userId}`);
    console.log(`[BLUETOOTH SCANNER]   - Storage path: ${storagePath}`);
    console.log(`[BLUETOOTH SCANNER]   - File size: ${fileContent.length} bytes`);
    
    // Mark file as uploaded in database
    await prisma.deviceFileIndex.create({
      data: {
        deviceId: dbDevice.id,
        fileName: fileName,
      },
    });
    
    console.log(`[BLUETOOTH SCANNER] Successfully processed ${fileName}`);
    
    // Try to delete the file from device after successful upload
    if (process.env.AUTO_DELETE_AFTER_UPLOAD === 'true') {
      try {
        // Extract session path from fileName (e.g., "20250117/7BF3655C/flight.dat" -> "20250117/7BF3655C")
        const sessionPath = fileName.substring(0, fileName.lastIndexOf('/'));
        
        console.log(`[BLUETOOTH SCANNER] Attempting to delete session ${sessionPath} from device...`);
        const deleteSuccess = await bluetooth.deleteSession(deviceName, sessionPath);
        
        if (deleteSuccess) {
          console.log(`[BLUETOOTH SCANNER] Successfully deleted session ${sessionPath} from device`);
        } else {
          console.warn(`[BLUETOOTH SCANNER] Failed to delete session ${sessionPath} from device`);
        }
      } catch (deleteError) {
        console.error(`[BLUETOOTH SCANNER] Error deleting file from device:`, deleteError);
        // Continue - deletion failure should not affect upload success
      }
    }
    
    return true;
    
  } catch (error) {
    console.error(`[BLUETOOTH SCANNER] Error processing file ${fileName}:`, error);
    
    // Don't mark as processed if there was an error
    return false;
  }
}

  private async updateDeviceStatuses() {
    try {
      const cutoffTime = new Date(Date.now() - DISCOVERY_WINDOW * 1000);
      
      console.log(`[BLUETOOTH SCANNER] Checking device statuses (cutoff: ${cutoffTime.toISOString()})`);
      
      // Get all non-provisioning devices
      const allDevices = await prisma.device.findMany({
        where: {
          state: {
            not: DeviceState.PROVISIONING,
          },
        },
        select: {
          id: true,
          name: true,
          bluetoothId: true,
          state: true,
          lastSeen: true,
        },
      });

      let updatedCount = 0;
      const updates = [];

      for (const device of allDevices) {
        const isOnline = device.lastSeen && device.lastSeen > cutoffTime;
        const currentlyActive = device.state === DeviceState.ACTIVE;
        
        if (!isOnline && currentlyActive) {
          // Mark as inactive
          await prisma.device.update({
            where: { id: device.id },
            data: { state: DeviceState.INACTIVE },
          });
          updatedCount++;
          updates.push(`${device.name} -> INACTIVE (last seen: ${device.lastSeen?.toISOString() || 'never'})`);
          console.log(`[BLUETOOTH SCANNER] Marked device ${device.name} as INACTIVE`);
        } else if (isOnline && !currentlyActive) {
          // This case is handled in processDevice when device is seen
          // Just log for visibility
          console.log(`[BLUETOOTH SCANNER] Device ${device.name} should be ACTIVE but isn't - will be fixed on next discovery`);
        }
      }

      if (updatedCount > 0) {
        console.log(`[BLUETOOTH SCANNER] Updated ${updatedCount} device statuses to INACTIVE`);
        updates.forEach(update => console.log(`[BLUETOOTH SCANNER]   - ${update}`));
      } else {
        console.log(`[BLUETOOTH SCANNER] All device statuses are correct`);
      }
      
      // Log summary of device states
      const stateSummary = await prisma.device.groupBy({
        by: ['state'],
        _count: true,
      });
      
      console.log(`[BLUETOOTH SCANNER] Device state summary:`);
      stateSummary.forEach(({ state, _count }) => {
        console.log(`[BLUETOOTH SCANNER]   - ${state}: ${_count} device(s)`);
      });
      
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