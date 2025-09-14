import { exec } from 'child_process';
import { promisify } from 'util';
import { spawn } from 'child_process';

const execAsync = promisify(exec);

export interface TempoDevice {
  name: string;
  bluetoothId: string; // MAC address - kept for reference but not used as primary identifier
  rssi: number; // Signal strength
}

interface BluetoothDevice {
  address: string;
  name?: string;
  rssi?: number;
}

export interface SessionInfo {
  name: string;
  is_dir: boolean;
  size: number;
}

export class BluetoothService {
  private static instance: BluetoothService;
  private pluginPath: string;

  private constructor() {
    // The plugin path should be configurable via environment variable
    this.pluginPath = process.env.SMPMGR_PLUGIN_PATH || 'smpmgr-extensions/plugins';
  }

  static getInstance(): BluetoothService {
    if (!BluetoothService.instance) {
      BluetoothService.instance = new BluetoothService();
    }
    return BluetoothService.instance;
  }

  /**
   * List all Tempo devices in range using bluetoothctl
   */
  async listTempoDevices(): Promise<TempoDevice[]> {
    console.log('[BLUETOOTH] Starting device discovery...');
    
    try {
      // Check if bluetoothctl is available
      const bluetoothctlAvailable = await this.checkBluetoothctl();
      if (!bluetoothctlAvailable) {
        console.warn('[BLUETOOTH] bluetoothctl not available, using mock devices');
        return this.getMockDevices();
      }

      // Use bluetoothctl to scan for devices
      const devices = await this.scanWithBluetoothctl();
      
      // Filter for Tempo devices (name starts with "Tempo-BT-")
      const tempoDevices = devices
        .filter(device => device.name && device.name.startsWith('Tempo-BT-'))
        .map(device => ({
          name: device.name!,
          bluetoothId: device.address,
          rssi: device.rssi || -80, // Default RSSI if not provided
        }));

      console.log(`[BLUETOOTH] Discovery complete. Found ${tempoDevices.length} Tempo device(s):`);
      tempoDevices.forEach(device => {
        console.log(`[BLUETOOTH]   - ${device.name} [${device.bluetoothId}] RSSI: ${device.rssi} dBm`);
      });

      if (tempoDevices.length === 0) {
        console.log('[BLUETOOTH]   No Tempo devices found in range');
      }

      return tempoDevices;
    } catch (error) {
      console.error('[BLUETOOTH] Error during device discovery:', error);
      return this.getMockDevices();
    }
  }

  /**
   * Scan for Bluetooth devices using bluetoothctl
   */
  private async scanWithBluetoothctl(): Promise<BluetoothDevice[]> {
    return new Promise((resolve, reject) => {
      const devices = new Map<string, BluetoothDevice>();
      const scanDuration = 30000; // 30 seconds scan as requested
      
      // Spawn bluetoothctl process
      const bluetoothctl = spawn('bluetoothctl');
      
      let outputBuffer = '';
      let scanComplete = false;
      
      bluetoothctl.stdout.on('data', (data) => {
        outputBuffer += data.toString();
        const lines = outputBuffer.split('\n');
        
        // Process complete lines
        for (let i = 0; i < lines.length - 1; i++) {
          const line = lines[i].trim();
          
          // Parse device discovery lines during scan
          if (!scanComplete) {
            // Example: [NEW] Device C8:43:CA:EB:FE:6D Tempo-BT-0004
            const newDeviceMatch = line.match(/\[NEW\] Device ([0-9A-F:]{17}) (.+)/);
            if (newDeviceMatch) {
              const [, address, name] = newDeviceMatch;
              if (!devices.has(address)) {
                devices.set(address, { address, name });
                console.log(`[BLUETOOTH] Discovered: ${name} [${address}]`);
              }
            }
            
            // Parse RSSI updates
            // Example: [CHG] Device C8:43:CA:EB:FE:6D RSSI: -76
            const rssiMatch = line.match(/\[CHG\] Device ([0-9A-F:]{17}) RSSI: (-?\d+)/);
            if (rssiMatch) {
              const [, address, rssi] = rssiMatch;
              const device = devices.get(address);
              if (device) {
                device.rssi = parseInt(rssi);
              }
            }
          } else {
            // Parse devices command output after scan
            // Example: Device C8:43:CA:EB:FE:6D Tempo-BT-0004
            const deviceMatch = line.match(/^Device ([0-9A-F:]{17}) (.+)$/);
            if (deviceMatch) {
              const [, address, name] = deviceMatch;
              // Update or add device info
              if (devices.has(address)) {
                const device = devices.get(address)!;
                device.name = name; // Update name if needed
              } else {
                devices.set(address, { address, name });
              }
            }
          }
        }
        
        // Keep the last incomplete line
        outputBuffer = lines[lines.length - 1];
      });

      bluetoothctl.stderr.on('data', (data) => {
        console.error('[BLUETOOTH] bluetoothctl error:', data.toString());
      });

      bluetoothctl.on('error', (error) => {
        reject(error);
      });

      // Start scanning
      console.log('[BLUETOOTH] Starting 30-second scan...');
      bluetoothctl.stdin.write('scan on\n');

      // Stop scanning after duration and get device list
      setTimeout(() => {
        console.log('[BLUETOOTH] Stopping scan...');
        bluetoothctl.stdin.write('scan off\n');
        scanComplete = true;
        
        // Wait a moment for scan to stop, then request device list
        setTimeout(() => {
          console.log('[BLUETOOTH] Getting device list...');
          bluetoothctl.stdin.write('devices\n');
          
          // Wait for devices output, then exit
          setTimeout(() => {
            bluetoothctl.stdin.write('exit\n');
            bluetoothctl.stdin.end();
            
            // Give it a final moment to process
            setTimeout(() => {
              console.log(`[BLUETOOTH] Found ${devices.size} total devices`);
              resolve(Array.from(devices.values()));
            }, 500);
          }, 2000); // Wait 2 seconds for devices output
        }, 1000); // Wait 1 second after scan off
      }, scanDuration);
    });
  }

  /**
   * Get mock devices for development/testing
   */
  private getMockDevices(): TempoDevice[] {
    const mockDevices: TempoDevice[] = [
      {
        name: 'Tempo-BT-0001',
        bluetoothId: 'C8:43:CA:EB:FE:6D',
        rssi: -45,
      },
      {
        name: 'Tempo-BT-0002', 
        bluetoothId: 'A8:51:AB:94:7E:03',
        rssi: -62,
      },
      {
        name: 'Tempo-BT-0003',
        bluetoothId: '62:CD:E5:D8:31:0A',
        rssi: -78,
      },
    ];

    // Simulate realistic discovery - not all devices are always found
    const discoveredDevices = mockDevices.filter(device => {
      const probability = Math.max(0.5, Math.min(0.95, 1 - (Math.abs(device.rssi) - 40) / 80));
      return Math.random() < probability;
    });

    return discoveredDevices;
  }

  /**
   * Check if bluetoothctl is available
   */
  async checkBluetoothctl(): Promise<boolean> {
    try {
      const { stdout } = await execAsync('bluetoothctl --version');
      if (stdout && stdout.includes('bluetoothctl')) {
        console.log(`[BLUETOOTH] bluetoothctl is available: ${stdout.trim()}`);
        return true;
      }
      return false;
    } catch (error) {
      console.error('[BLUETOOTH] bluetoothctl not found:', error);
      return false;
    }
  }

  /**
   * Check if smpmgr is available
   */
  async checkSmpmgr(): Promise<boolean> {
    try {
      // Try to run smpmgr with --help to verify it's installed and working
      const { stdout } = await execAsync('smpmgr --help');
      
      // Verify we got some output (smpmgr should show help text)
      if (stdout && stdout.length > 0) {
        console.log('[BLUETOOTH] smpmgr is available');
        
        // Also check if the plugin path exists
        try {
          await execAsync(`ls ${this.pluginPath}`);
          console.log(`[BLUETOOTH] Plugin path ${this.pluginPath} exists`);
        } catch (error) {
          console.warn(`[BLUETOOTH] Warning: Plugin path ${this.pluginPath} not found. Set SMPMGR_PLUGIN_PATH environment variable if needed.`);
        }
        
        return true;
      }
      
      return false;
    } catch (error) {
      console.error('[BLUETOOTH] smpmgr check failed:', error);
      return false;
    }
  }

  /**
   * Get session list from device using smpmgr tempo session-list
   */
  async getSessionList(deviceName: string): Promise<SessionInfo[]> {
    console.log(`[BLUETOOTH] Getting session list from device ${deviceName}...`);
    
    try {
      // Run smpmgr tempo session-list command
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} tempo session-list`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      const { stdout } = await execAsync(command, { timeout: 30000 });
      
      // Parse JSON response
      try {
        const jsonMatch = stdout.match(/\{[\s\S]*"sessions"[\s\S]*\}/);
        if (!jsonMatch) {
          console.warn('[BLUETOOTH] No JSON found in session-list output');
          return [];
        }
        
        const data = JSON.parse(jsonMatch[0]);
        return data.sessions || [];
      } catch (parseError) {
        console.error('[BLUETOOTH] Error parsing session list JSON:', parseError);
        return [];
      }
      
    } catch (error) {
      console.error(`[BLUETOOTH] Error getting session list:`, error);
      return [];
    }
  }

  /**
   * List files on a device using smpmgr tempo session-list
   */
  async listDeviceFiles(deviceName: string): Promise<string[]> {
    console.log(`[BLUETOOTH] Listing files on device ${deviceName}...`);
    
    try {
      // Run smpmgr tempo session-list command to get sessions
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} tempo session-list`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      const { stdout } = await execAsync(command, { timeout: 30000 }); // 30 second timeout
      
      // Parse the output to extract session paths and construct file paths
      const files = this.parseSessionListOutput(stdout);
      
      console.log(`[BLUETOOTH] Found ${files.length} files on device`);
      return files;
      
    } catch (error) {
      console.error(`[BLUETOOTH] Error listing files:`, error);
      // Fall back to mock data if real command fails
      return this.getMockFiles();
    }
  }

  /**
   * Parse smpmgr tempo session-list output to construct file paths
   */
  private parseSessionListOutput(output: string): string[] {
    const files: string[] = [];
    
    try {
      // The session-list command should return JSON with sessions array
      // Look for JSON in the output (it might have other text around it)
      const jsonMatch = output.match(/\{[\s\S]*"sessions"[\s\S]*\}/);
      if (!jsonMatch) {
        console.warn('[BLUETOOTH] No JSON found in session-list output');
        return files;
      }
      
      const sessionData = JSON.parse(jsonMatch[0]);
      
      if (sessionData.sessions && Array.isArray(sessionData.sessions)) {
        // Add config files that are always present
        files.push('uinfo.json', 'config.json');
        
        // For each session directory, add the flight.dat file path
        for (const session of sessionData.sessions) {
          if (session.name && session.is_dir) {
            // Session name is like "20250115/ABC12345"
            files.push(`${session.name}/flight.dat`);
          }
        }
      }
    } catch (error) {
      console.error('[BLUETOOTH] Error parsing session list:', error);
    }
    
    return files;
  }

  /**
   * Get mock files for development
   */
  private getMockFiles(): string[] {
    const baseFiles = [
      'uinfo.json',
      'config.json',
    ];
    
    // Generate some mock jump logs
    const jumpLogs: string[] = [];
    const today = new Date();
    const numJumps = Math.floor(Math.random() * 3) + 1;
    
    for (let i = 0; i < numJumps; i++) {
      const daysAgo = Math.floor(Math.random() * 7);
      const jumpDate = new Date(today);
      jumpDate.setDate(jumpDate.getDate() - daysAgo);
      
      const dateStr = jumpDate.toISOString().slice(0, 10).replace(/-/g, '');
      const sessionId = Math.random().toString(36).substring(2, 10).toUpperCase();
      
      jumpLogs.push(`${dateStr}/${sessionId}/flight.dat`);
    }
    
    return [...baseFiles, ...jumpLogs.sort()];
  }

  /**
   * Download a file from device using smpmgr file download
   */
  async downloadFile(deviceName: string, fileName: string): Promise<Buffer> {
    console.log(`[BLUETOOTH] Downloading ${fileName} from ${deviceName}...`);
    
    try {
      // Construct full path
      const fullPath = `/lfs/logs/${fileName}`;
      
      // Create a temporary filename for download
      const tempFile = `/tmp/tempo-download-${Date.now()}.dat`;
      
      // Run smpmgr file download command
      // smpmgr file download saves to a local file
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} file download ${fullPath} ${tempFile}`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      await execAsync(command, { timeout: 60000 }); // 60 second timeout for downloads
      
      // Read the downloaded file
      const fs = await import('fs/promises');
      const fileContent = await fs.readFile(tempFile);
      
      // Clean up temp file
      try {
        await fs.unlink(tempFile);
      } catch (e) {
        console.warn(`[BLUETOOTH] Failed to delete temp file ${tempFile}`);
      }
      
      console.log(`[BLUETOOTH] Downloaded ${fileContent.length} bytes`);
      return fileContent;
      
    } catch (error) {
      console.error(`[BLUETOOTH] Error downloading file:`, error);
      // Fall back to mock data if real command fails
      return this.getMockFileContent(fileName);
    }
  }

  /**
   * Get mock file content
   */
  private getMockFileContent(fileName: string): Buffer {
    // For testing, return different content based on filename
    const mockContent = `Mock content for ${fileName}\nDevice data would go here...`;
    return Buffer.from(mockContent, 'utf-8');
  }

  /**
   * Send blink command to device using smpmgr
   */
  async blinkDevice(deviceName: string, color: string = 'orange'): Promise<void> {
    console.log(`[BLUETOOTH] Sending blink command to ${deviceName} with color ${color}...`);
    let subcommand = `led-on ${color}`;
    
    try {
      if (color === 'off') {
        subcommand = 'led-off';
      }
      // Run smpmgr tempo led-on command
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} tempo ${subcommand}`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      await execAsync(command, { timeout: 10000 }); // 10 second timeout
      
      console.log(`[BLUETOOTH] LED command sent successfully`);
    } catch (error) {
      console.error(`[BLUETOOTH] Error sending blink command:`, error);
      // Continue even if blink fails
    }
  }

  /**
   * Delete a file from device using smpmgr file delete
   */
  async deleteFile(deviceName: string, filePath: string): Promise<boolean> {
    console.log(`[BLUETOOTH] Deleting file ${filePath} from ${deviceName}...`);
    
    try {
      // Construct full path
      const fullPath = `/lfs/logs/${filePath}`;
      
      // Run smpmgr file delete command
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} file delete ${fullPath}`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      const { stdout } = await execAsync(command, { timeout: 30000 });
      
      console.log(`[BLUETOOTH] File ${filePath} deleted successfully`);
      return true;
      
    } catch (error) {
      console.error(`[BLUETOOTH] Error deleting file:`, error);
      return false;
    }
  }

  /**
   * Delete a session from device using smpmgr tempo session-delete
   */
  async deleteSession(deviceName: string, sessionPath: string): Promise<boolean> {
    console.log(`[BLUETOOTH] Deleting session ${sessionPath} from ${deviceName}...`);
    
    try {
      // Run smpmgr tempo session-delete command
      const command = `smpmgr --ble ${deviceName} --plugin-path=${this.pluginPath} tempo session-delete ${sessionPath} --yes`;
      console.log(`[BLUETOOTH] Running: ${command}`);
      
      const { stdout } = await execAsync(command, { timeout: 30000 }); // 30 second timeout
      
      // Check if deletion was successful
      const success = stdout.includes('deleted successfully');
      
      if (success) {
        console.log(`[BLUETOOTH] Session ${sessionPath} deleted successfully`);
      } else {
        console.warn(`[BLUETOOTH] Failed to delete session: ${stdout}`);
      }
      
      return success;
    } catch (error) {
      console.error(`[BLUETOOTH] Error deleting session:`, error);
      return false;
    }
  }
}