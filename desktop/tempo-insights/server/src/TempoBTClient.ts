import { exec } from 'child_process';
import { promisify } from 'util';
import * as path from 'path';
import * as fs from 'fs/promises';

const execAsync = promisify(exec);

/**
 * Device information returned by device discovery
 */
export interface BluetoothDevice {
  name: string;
  address: string;
  rssi?: number;
}

/**
 * Storage information structure
 */
export interface StorageInfo {
  freeBytes: number;
  totalBytes: number;
  backend: string;
}

/**
 * Device settings structure
 */
export interface DeviceSettings {
  bleName?: string;
  userUuid?: string;
  deviceUuid?: string;
  logBackend?: string;
  ppsEnabled?: boolean;
}

/**
 * File information structure
 */
export interface FileInfo {
  name: string;
  size: number;
  isDirectory: boolean;
}

/**
 * Logger state enumeration
 */
export enum LoggerState {
  IDLE = "IDLE",
  ARMED = "ARMED",
  LOGGING = "LOGGING",
  ERROR = "ERROR"
}

/**
 * TempoBTClient - A TypeScript wrapper for mcumgr commands to interact with Tempo-BT devices
 */
export class TempoBTClient {
  private deviceName: string;
  private connectionString: string;
  private mcumgrPath: string;
  private timeout: number;

  /**
   * Creates a new TempoBTClient instance
   * @param deviceName - The Bluetooth name of the device (default: "Tempo-BT")
   * @param mcumgrPath - Path to mcumgr executable (default: "mcumgr" - assumes it's in PATH)
   * @param timeout - Command timeout in seconds (default: 10)
   */
  constructor(deviceName: string = "Tempo-BT", mcumgrPath: string = "mcumgr", timeout: number = 10) {
    this.deviceName = deviceName;
    this.connectionString = `peer_name='${deviceName}'`;
    this.mcumgrPath = mcumgrPath;
    this.timeout = timeout;
  }

  /**
   * Execute mcumgr command with proper error handling
   */
  private async execMcumgr(args: string): Promise<string> {
    const command = `${this.mcumgrPath} --conntype ble --connstring ${this.connectionString} -t ${this.timeout} ${args}`;
    
    try {
      const { stdout, stderr } = await execAsync(command);
      if (stderr && !stderr.includes("Warning")) {
        throw new Error(`mcumgr error: ${stderr}`);
      }
      return stdout.trim();
    } catch (error) {
      if (error instanceof Error) {
        throw new Error(`Failed to execute mcumgr: ${error.message}`);
      }
      throw error;
    }
  }

  /**
   * Find available Tempo-BT devices
   * @param controllerName - Bluetooth controller name (default: "hci0")
   * @returns Array of discovered devices
   */
  public async findDevices(controllerName: string = "hci0"): Promise<BluetoothDevice[]> {
    try {
      const command = `${this.mcumgrPath} --conntype ble --connstring ctlr_name=${controllerName} conn find`;
      const { stdout } = await execAsync(command);
      
      // Parse the output to extract device information
      const devices: BluetoothDevice[] = [];
      const lines = stdout.split('\n');
      
      for (const line of lines) {
        // Parse lines that contain device information
        // Format typically: "peer_name='DeviceName' addr=XX:XX:XX:XX:XX:XX"
        const nameMatch = line.match(/peer_name='([^']+)'/);
        const addrMatch = line.match(/addr=([A-Fa-f0-9:]+)/);
        const rssiMatch = line.match(/rssi=(-?\d+)/);
        
        if (nameMatch && addrMatch) {
          devices.push({
            name: nameMatch[1],
            address: addrMatch[1],
            rssi: rssiMatch ? parseInt(rssiMatch[1]) : undefined
          });
        }
      }
      
      return devices;
    } catch (error) {
      throw new Error(`Failed to find devices: ${error}`);
    }
  }

  /**
   * Test device connection
   * @returns True if connection successful
   */
  public async testConnection(): Promise<boolean> {
    try {
      const result = await this.execMcumgr("echo hello");
      return result.includes("hello");
    } catch (error) {
      return false;
    }
  }

  /**
   * List files in a directory
   * @param path - Directory path (default: "/logs")
   * @returns Array of file information
   */
  public async listFiles(dirPath: string = "/logs"): Promise<FileInfo[]> {
    try {
      const result = await this.execMcumgr(`fs ls ${dirPath}`);
      const files: FileInfo[] = [];
      const lines = result.split('\n');
      
      for (const line of lines) {
        // Parse file listing output
        // Format typically: "filename size" or "dirname/"
        const match = line.match(/^\s*(\S+)\s+(\d+)?/);
        if (match) {
          const name = match[1];
          const size = match[2] ? parseInt(match[2]) : 0;
          const isDirectory = name.endsWith('/');
          
          files.push({
            name: isDirectory ? name.slice(0, -1) : name,
            size,
            isDirectory
          });
        }
      }
      
      return files;
    } catch (error) {
      throw new Error(`Failed to list files: ${error}`);
    }
  }

  /**
   * Download a file from the device
   * @param remotePath - Path on device
   * @param localPath - Local path to save file
   */
  public async downloadFile(remotePath: string, localPath: string): Promise<void> {
    try {
      await this.execMcumgr(`fs download ${remotePath} ${localPath}`);
    } catch (error) {
      throw new Error(`Failed to download file: ${error}`);
    }
  }

  /**
   * Delete a file on the device
   * @param remotePath - Path on device
   */
  public async deleteFile(remotePath: string): Promise<void> {
    try {
      await this.execMcumgr(`fs delete ${remotePath}`);
    } catch (error) {
      throw new Error(`Failed to delete file: ${error}`);
    }
  }

  /**
   * Get device statistics
   * @returns Raw statistics output
   */
  public async getStatistics(): Promise<string> {
    try {
      return await this.execMcumgr("stat list");
    } catch (error) {
      throw new Error(`Failed to get statistics: ${error}`);
    }
  }

  /**
   * Get storage information using custom command
   * @returns Storage information
   */
  public async getStorageInfo(): Promise<StorageInfo> {
    try {
      const result = await this.execMcumgr("custom storage_info -g 64");
      
      // Parse the custom command response
      // This would need to be adjusted based on actual response format
      const freeMatch = result.match(/free[:\s]+(\d+)/i);
      const totalMatch = result.match(/total[:\s]+(\d+)/i);
      const backendMatch = result.match(/backend[:\s]+(\w+)/i);
      
      return {
        freeBytes: freeMatch ? parseInt(freeMatch[1]) : 0,
        totalBytes: totalMatch ? parseInt(totalMatch[1]) : 0,
        backend: backendMatch ? backendMatch[1] : "unknown"
      };
    } catch (error) {
      throw new Error(`Failed to get storage info: ${error}`);
    }
  }

  /**
   * Read a device setting
   * @param setting - Setting name (e.g., "app/ble_name")
   * @returns Setting value
   */
  public async readSetting(setting: string): Promise<string> {
    try {
      const result = await this.execMcumgr(`config ${setting}`);
      
      // Parse the response to extract the value
      // Format typically: "app/ble_name: value"
      const match = result.match(/:\s*(.+)$/m);
      return match ? match[1].trim() : "";
    } catch (error) {
      throw new Error(`Failed to read setting ${setting}: ${error}`);
    }
  }

  /**
   * Write a device setting
   * @param setting - Setting name
   * @param value - New value
   */
  public async writeSetting(setting: string, value: string): Promise<void> {
    try {
      await this.execMcumgr(`config ${setting} set "${value}"`);
    } catch (error) {
      throw new Error(`Failed to write setting ${setting}: ${error}`);
    }
  }

  /**
   * Get all device settings
   * @returns Device settings object
   */
  public async getAllSettings(): Promise<DeviceSettings> {
    try {
      const result = await this.execMcumgr("config app");
      
      const settings: DeviceSettings = {};
      
      // Parse each setting from the response
      const bleNameMatch = result.match(/app\/ble_name:\s*(.+)$/m);
      if (bleNameMatch) settings.bleName = bleNameMatch[1].trim();
      
      const userUuidMatch = result.match(/app\/user_uuid:\s*([a-fA-F0-9-]+)$/m);
      if (userUuidMatch) settings.userUuid = userUuidMatch[1].trim();
      
      const deviceUuidMatch = result.match(/app\/device_uuid:\s*([a-fA-F0-9-]+)$/m);
      if (deviceUuidMatch) settings.deviceUuid = deviceUuidMatch[1].trim();
      
      const logBackendMatch = result.match(/app\/log_backend:\s*(\w+)$/m);
      if (logBackendMatch) settings.logBackend = logBackendMatch[1].trim();
      
      const ppsEnabledMatch = result.match(/app\/pps_enabled:\s*(true|false)$/m);
      if (ppsEnabledMatch) settings.ppsEnabled = ppsEnabledMatch[1] === "true";
      
      return settings;
    } catch (error) {
      throw new Error(`Failed to get all settings: ${error}`);
    }
  }

  /**
   * Set device Bluetooth name
   * @param name - New Bluetooth name (max 31 chars)
   */
  public async setBleName(name: string): Promise<void> {
    if (name.length > 31) {
      throw new Error("Bluetooth name must be 31 characters or less");
    }
    await this.writeSetting("app/ble_name", name);
  }

  /**
   * Set user UUID
   * @param uuid - UUID in standard format or 32-char hex
   */
  public async setUserUuid(uuid: string): Promise<void> {
    const hexUuid = this.formatUuidForMcumgr(uuid);
    await this.writeSetting("app/user_uuid", hexUuid);
  }

  /**
   * Set device UUID
   * @param uuid - UUID in standard format or 32-char hex
   */
  public async setDeviceUuid(uuid: string): Promise<void> {
    const hexUuid = this.formatUuidForMcumgr(uuid);
    await this.writeSetting("app/device_uuid", hexUuid);
  }

  /**
   * Set log backend
   * @param backend - "littlefs" or "fatfs"
   */
  public async setLogBackend(backend: "littlefs" | "fatfs"): Promise<void> {
    await this.writeSetting("app/log_backend", backend);
  }

  /**
   * Download all log files from a specific date
   * @param date - Date in YYYYMMDD format
   * @param localDir - Local directory to save files
   * @returns Array of downloaded file paths
   */
  public async downloadLogsByDate(date: string, localDir: string): Promise<string[]> {
    const downloadedFiles: string[] = [];
    
    try {
      // List sessions for the date
      const sessions = await this.listFiles(`/logs/${date}`);
      
      for (const session of sessions) {
        if (session.isDirectory) {
          const logPath = `/logs/${date}/${session.name}/flight.csv`;
          const localPath = path.join(localDir, `${date}_${session.name}_flight.csv`);
          
          try {
            await this.downloadFile(logPath, localPath);
            downloadedFiles.push(localPath);
          } catch (error) {
            console.error(`Failed to download ${logPath}: ${error}`);
          }
        }
      }
      
      return downloadedFiles;
    } catch (error) {
      throw new Error(`Failed to download logs for date ${date}: ${error}`);
    }
  }

  /**
   * Format UUID for mcumgr (remove dashes)
   */
  private formatUuidForMcumgr(uuid: string): string {
    return uuid.replace(/-/g, '');
  }

  /**
   * Set connection timeout
   * @param seconds - Timeout in seconds
   */
  public setConnectionTimeout(seconds: number): void {
    this.timeout = seconds;
  }

  /**
   * Change target device name
   * @param deviceName - New device name
   */
  public setDeviceName(deviceName: string): void {
    this.deviceName = deviceName;
    this.connectionString = `peer_name='${deviceName}'`;
  }
}