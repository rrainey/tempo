import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export interface TempoDevice {
  name: string;
  bluetoothId: string; // MAC address
  rssi: number; // Signal strength
  isTempoDevice: boolean;
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
   * List all Tempo devices in range
   * TODO: Implement actual BLE scanning with bluetoothctl
   */
  async listTempoDevices(): Promise<TempoDevice[]> {
    console.log('[BLUETOOTH] Scanning for Tempo devices...');
    
    // TODO: Replace with actual bluetoothctl device discovery
    // For now, return mocked devices for development
    const mockDevices: TempoDevice[] = [
      {
        name: 'Tempo-BT-0001',
        bluetoothId: 'AA:BB:CC:DD:EE:01',
        rssi: -45,
        isTempoDevice: true,
      },
      {
        name: 'Tempo-BT-0002', 
        bluetoothId: 'AA:BB:CC:DD:EE:02',
        rssi: -62,
        isTempoDevice: true,
      },
      {
        name: 'Tempo-BT-0003',
        bluetoothId: 'AA:BB:CC:DD:EE:03',
        rssi: -78,
        isTempoDevice: true,
      },
    ];

    // Simulate scan delay
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Randomly include only some devices to simulate range
    const discoveredDevices = mockDevices.filter(() => Math.random() > 0.3);
    
    console.log(`[BLUETOOTH] Found ${discoveredDevices.length} Tempo devices`);
    discoveredDevices.forEach(device => {
      console.log(`[BLUETOOTH]   - ${device.name} (${device.bluetoothId}) RSSI: ${device.rssi}`);
    });

    return discoveredDevices;
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
   * Check if mcumgr is available (deprecated - use checkSmpmgr instead)
   * @deprecated Use checkSmpmgr() instead
   */
  async checkMcumgr(): Promise<boolean> {
    console.warn('[BLUETOOTH] checkMcumgr() is deprecated, use checkSmpmgr() instead');
    return this.checkSmpmgr();
  }

  /**
   * List files on a device
   * TODO: Implement smpmgr fs ls
   */
  async listDeviceFiles(bluetoothId: string): Promise<string[]> {
    console.log(`[BLUETOOTH] Listing files on device ${bluetoothId}...`);
    
    // Mock file list for development
    const mockFiles = [
      'jump_001.dat',
      'jump_002.dat', 
      'jump_003.dat',
      'uinfo.json',
      'config.json',
    ];

    await new Promise(resolve => setTimeout(resolve, 1000));
    
    return mockFiles;
  }

  /**
   * Download a file from device
   * TODO: Implement smpmgr fs download
   */
  async downloadFile(bluetoothId: string, fileName: string): Promise<Buffer> {
    console.log(`[BLUETOOTH] Downloading ${fileName} from ${bluetoothId}...`);
    
    // Mock file content
    const mockContent = Buffer.from(`Mock content for ${fileName} from device ${bluetoothId}`);
    
    await new Promise(resolve => setTimeout(resolve, 1500));
    
    return mockContent;
  }

  /**
   * Send blink command to device
   * TODO: Implement actual command using smpmgr tempo led-on
   */
  async blinkDevice(bluetoothId: string): Promise<void> {
    console.log(`[BLUETOOTH] Sending blink command to ${bluetoothId}...`);
    await new Promise(resolve => setTimeout(resolve, 500));
  }
}