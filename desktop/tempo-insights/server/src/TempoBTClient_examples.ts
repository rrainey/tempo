import { TempoBTClient, LoggerState } from './TempoBTClient';

async function main() {
  // Create client instance
  const client = new TempoBTClient('Tempo-BT', 'mcumgr', 10);
  
  try {
    // 1. Find available devices
    console.log('Searching for devices...');
    const devices = await client.findDevices('hci0');
    console.log('Found devices:', devices);
    
    // 2. Test connection
    console.log('\nTesting connection...');
    const connected = await client.testConnection();
    if (!connected) {
      throw new Error('Failed to connect to device');
    }
    console.log('Connection successful!');
    
    // 3. Get device information
    console.log('\nReading device settings...');
    const settings = await client.getAllSettings();
    console.log('Device settings:', settings);
    
    // 4. Get storage info
    console.log('\nGetting storage information...');
    const storage = await client.getStorageInfo();
    console.log(`Storage: ${storage.freeBytes / (1024*1024)} MB free of ${storage.totalBytes / (1024*1024)} MB`);
    
    // 5. List log files
    console.log('\nListing log files...');
    const files = await client.listFiles('/logs');
    console.log('Log directories:', files);
    
    // 6. Download today's logs
    const today = new Date().toISOString().split('T')[0].replace(/-/g, '');
    console.log(`\nDownloading logs for ${today}...`);
    const downloadedFiles = await client.downloadLogsByDate(today, './downloads');
    console.log('Downloaded files:', downloadedFiles);
    
    // 7. Update device settings
    console.log('\nUpdating device name...');
    await client.setBleName('MyTempo');
    
    // 8. Set log backend to SD card if available
    console.log('Setting log backend to SD card...');
    await client.setLogBackend('fatfs');
    
  } catch (error) {
    console.error('Error:', error);
  }
}

// Advanced usage example with error handling and retries
class TempoBTManager {
  private client: TempoBTClient;
  private maxRetries: number = 3;
  
  constructor(deviceName: string = 'Tempo-BT') {
    this.client = new TempoBTClient(deviceName);
  }
  
  /**
   * Connect with automatic retry
   */
  async connectWithRetry(): Promise<boolean> {
    for (let i = 0; i < this.maxRetries; i++) {
      try {
        const connected = await this.client.testConnection();
        if (connected) return true;
        
        console.log(`Connection attempt ${i + 1} failed, retrying...`);
        await this.delay(2000); // Wait 2 seconds between retries
      } catch (error) {
        console.error(`Connection error on attempt ${i + 1}:`, error);
      }
    }
    return false;
  }
  
  /**
   * Monitor device and download new logs periodically
   */
  async monitorDevice(intervalMinutes: number = 5): Promise<void> {
    console.log(`Starting device monitoring (checking every ${intervalMinutes} minutes)...`);
    
    while (true) {
      try {
        // Check connection
        if (!await this.client.testConnection()) {
          console.log('Device disconnected, attempting reconnection...');
          if (!await this.connectWithRetry()) {
            console.error('Failed to reconnect, will retry later...');
            await this.delay(60000); // Wait 1 minute before next attempt
            continue;
          }
        }
        
        // Get storage info
        const storage = await this.client.getStorageInfo();
        const freePercent = (storage.freeBytes / storage.totalBytes) * 100;
        console.log(`Storage: ${freePercent.toFixed(1)}% free`);
        
        // Check for new logs
        const today = new Date().toISOString().split('T')[0].replace(/-/g, '');
        const logs = await this.client.listFiles(`/logs/${today}`);
        console.log(`Found ${logs.length} sessions today`);
        
        // Download any new logs
        // (In a real implementation, you'd track which logs have been downloaded)
        
      } catch (error) {
        console.error('Monitoring error:', error);
      }
      
      await this.delay(intervalMinutes * 60 * 1000);
    }
  }
  
  /**
   * Batch operations on multiple log files
   */
  async downloadAllLogs(outputDir: string): Promise<void> {
    try {
      // List all date directories
      const dateDirs = await this.client.listFiles('/logs');
      
      for (const dateDir of dateDirs) {
        if (dateDir.isDirectory && /^\d{8}$/.test(dateDir.name)) {
          console.log(`Processing logs for ${dateDir.name}...`);
          const files = await this.client.downloadLogsByDate(dateDir.name, outputDir);
          console.log(`  Downloaded ${files.length} files`);
        }
      }
    } catch (error) {
      throw new Error(`Failed to download all logs: ${error}`);
    }
  }
  
  /**
   * Configure device for specific user
   */
  async configureForUser(userName: string, userUuid?: string): Promise<void> {
    try {
      // Set user-friendly device name
      await this.client.setBleName(`${userName}-Tempo`);
      
      // Set user UUID if provided
      if (userUuid) {
        await this.client.setUserUuid(userUuid);
      }
      
      // Ensure SD card is used if available
      await this.client.setLogBackend('fatfs');
      
      console.log(`Device configured for user: ${userName}`);
    } catch (error) {
      throw new Error(`Failed to configure device: ${error}`);
    }
  }
  
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// Run the examples
if (require.main === module) {
  main().catch(console.error);
}