import { BluetoothService } from '../bluetooth.service';
import { exec } from 'child_process';
import { promisify } from 'util';

// Mock the exec function
jest.mock('child_process');
jest.mock('util', () => ({
  ...jest.requireActual('util'),
  promisify: jest.fn((fn) => fn),
}));

describe('BluetoothService', () => {
  let bluetoothService: BluetoothService;
  let mockExec: jest.MockedFunction<typeof exec>;

  beforeEach(() => {
    bluetoothService = BluetoothService.getInstance();
    mockExec = exec as jest.MockedFunction<typeof exec>;
    jest.clearAllMocks();
  });

  describe('checkSmpmgr', () => {
    it('should return true when smpmgr is available', async () => {
      // Mock successful smpmgr execution
      mockExec.mockImplementation((command, callback) => {
        if (command === 'smpmgr --help') {
          callback(null, { stdout: 'Usage: smpmgr [OPTIONS] COMMAND [ARGS]...', stderr: '' } as any);
        } else if (command.startsWith('ls')) {
          callback(null, { stdout: 'plugins', stderr: '' } as any);
        }
      });

      const result = await bluetoothService.checkSmpmgr();
      
      expect(result).toBe(true);
      expect(mockExec).toHaveBeenCalledWith('smpmgr --help');
    });

    it('should return false when smpmgr is not available', async () => {
      // Mock failed smpmgr execution
      mockExec.mockImplementation((command, callback) => {
        callback(new Error('command not found: smpmgr'), { stdout: '', stderr: 'command not found' } as any);
      });

      const result = await bluetoothService.checkSmpmgr();
      
      expect(result).toBe(false);
      expect(mockExec).toHaveBeenCalledWith('smpmgr --help');
    });

    it('should return false when smpmgr returns no output', async () => {
      // Mock smpmgr returning empty output
      mockExec.mockImplementation((command, callback) => {
        callback(null, { stdout: '', stderr: '' } as any);
      });

      const result = await bluetoothService.checkSmpmgr();
      
      expect(result).toBe(false);
    });

    it('should warn when plugin path does not exist', async () => {
      const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();
      
      // Mock successful smpmgr but failed plugin path check
      mockExec.mockImplementation((command, callback) => {
        if (command === 'smpmgr --help') {
          callback(null, { stdout: 'Usage: smpmgr...', stderr: '' } as any);
        } else if (command.startsWith('ls')) {
          callback(new Error('No such file or directory'), { stdout: '', stderr: '' } as any);
        }
      });

      const result = await bluetoothService.checkSmpmgr();
      
      expect(result).toBe(true); // Should still return true
      expect(consoleSpy).toHaveBeenCalledWith(expect.stringContaining('Warning: Plugin path'));
      
      consoleSpy.mockRestore();
    });
  });

  describe('checkMcumgr (deprecated)', () => {
    it('should call checkSmpmgr and warn about deprecation', async () => {
      const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();
      const checkSmpmgrSpy = jest.spyOn(bluetoothService, 'checkSmpmgr').mockResolvedValue(true);
      
      const result = await bluetoothService.checkMcumgr();
      
      expect(result).toBe(true);
      expect(checkSmpmgrSpy).toHaveBeenCalled();
      expect(consoleSpy).toHaveBeenCalledWith('[BLUETOOTH] checkMcumgr() is deprecated, use checkSmpmgr() instead');
      
      consoleSpy.mockRestore();
      checkSmpmgrSpy.mockRestore();
    });
  });
});