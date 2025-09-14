import { PrismaClient } from '@prisma/client';
import { config } from 'dotenv';
import { LogParser } from '@/lib/analysis/log-processor';

// Load environment variables
config({ path: '.env' });

const prisma = new PrismaClient();

// Configuration
const PROCESS_INTERVAL = 30000; // 30 seconds between processing cycles
const BATCH_SIZE = 10; // Process up to 10 logs per cycle

class LogProcessor {
  private isRunning = false;
  private processInterval: NodeJS.Timeout | null = null;
  private cycleCount = 0;

  async start() {
    console.log('[LOG PROCESSOR] Starting worker...');
    console.log(`[LOG PROCESSOR] Configuration:`);
    console.log(`[LOG PROCESSOR]   - Process interval: ${PROCESS_INTERVAL / 1000} seconds`);
    console.log(`[LOG PROCESSOR]   - Batch size: ${BATCH_SIZE} logs per cycle`);
    
    this.isRunning = true;

    // Initial processing
    await this.processCycle();

    // Set up interval
    this.processInterval = setInterval(async () => {
      if (this.isRunning) {
        await this.processCycle();
      }
    }, PROCESS_INTERVAL);

    // Handle shutdown gracefully
    process.on('SIGINT', () => this.stop());
    process.on('SIGTERM', () => this.stop());
  }

  async stop() {
    console.log('[LOG PROCESSOR] Stopping worker...');
    this.isRunning = false;
    
    if (this.processInterval) {
      clearInterval(this.processInterval);
    }

    await prisma.$disconnect();
    process.exit(0);
  }

  private async processCycle() {
    this.cycleCount++;
    const cycleId = `${Date.now()}-${this.cycleCount}`;
    
    console.log(`[LOG PROCESSOR] Starting cycle ${this.cycleCount} (ID: ${cycleId})...`);
    const startTime = Date.now();
    
    try {
      // TODO: Queue query - Task 65
      // TODO: Process logs - Tasks 66-73
      
      // For now, just log that we're running
      const pendingCount = await this.getPendingLogCount();
      
      if (pendingCount > 0) {
        console.log(`[LOG PROCESSOR] Found ${pendingCount} pending log(s) to process`);
        // Processing will be implemented in subsequent tasks
      } else {
        console.log(`[LOG PROCESSOR] No pending logs to process`);
      }
      
      const duration = Date.now() - startTime;
      console.log(`[LOG PROCESSOR] Cycle ${this.cycleCount} completed in ${duration}ms`);
      
    } catch (error) {
      console.error(`[LOG PROCESSOR] Error in cycle ${this.cycleCount}:`, error);
    }
    
    console.log(`[LOG PROCESSOR] Cycle end\n`);
  }

  private async getPendingLogCount(): Promise<number> {
    // For now, count all logs as "pending" since we haven't added analysis fields yet
    // This will be updated in Task 65 after schema migration
    const count = await prisma.jumpLog.count();
    return count;
  }

  private async getQueuedLogs() {
    // Task 65: Get unprocessed logs ordered by newest first
    return await prisma.jumpLog.findMany({
      where: {
        initialAnalysisTimestamp: null
      },
      orderBy: {
        createdAt: 'desc'
      },
      take: BATCH_SIZE,
      include: {
        device: true,
        user: true
      }
    });
  }

  private async processJumpLog(jumpLog: any) {
    console.log(`[LOG PROCESSOR] Processing jump log ${jumpLog.id}`);
    console.log(`[LOG PROCESSOR]   - Created: ${jumpLog.createdAt.toISOString()}`);
    console.log(`[LOG PROCESSOR]   - User: ${jumpLog.user.name || jumpLog.user.email}`);
    console.log(`[LOG PROCESSOR]   - Device: ${jumpLog.device.name}`);
    console.log(`[LOG PROCESSOR]   - Size: ${jumpLog.rawLog.length} bytes`);
    
    try {
      // Task 66: Parse the log
      const validation = LogParser.validateLog(jumpLog.rawLog);
      if (!validation.isValid) {
        throw new Error(`Invalid log: ${validation.message}`);
      }
      
      const parsedData = LogParser.parseLog(jumpLog.rawLog);
      console.log(`[LOG PROCESSOR]   - Parsed: ${parsedData.altitude.length} altitude points`);
      console.log(`[LOG PROCESSOR]   - Duration: ${parsedData.duration}s`);
      console.log(`[LOG PROCESSOR]   - GPS: ${parsedData.hasGPS ? 'Yes' : 'No'}`);
      
      // TODO: Tasks 67-73 - Detect events and calculate metrics
      
      // For now, just mark as processed with basic info
      await prisma.jumpLog.update({
        where: { id: jumpLog.id },
        data: {
          initialAnalysisTimestamp: new Date(),
          initialAnalysisMessage: parsedData.hasGPS ? 'Processed with GPS' : 'Processed without GPS',
          // Store some basic offsets for testing
          offsets: {
            dataPoints: parsedData.altitude.length,
            duration: parsedData.duration,
            sampleRate: parsedData.sampleRate
          }
        }
      });
      
      console.log(`[LOG PROCESSOR]   ✓ Analysis complete`);
      
    } catch (error) {
      console.error(`[LOG PROCESSOR]   ✗ Error processing log:`, error);
      
      // Mark as processed with error
      await prisma.jumpLog.update({
        where: { id: jumpLog.id },
        data: {
          initialAnalysisTimestamp: new Date(),
          initialAnalysisMessage: `Processing error: ${error instanceof Error ? error.message : 'Unknown error'}`
        }
      });
    }
  }
}

// Start the processor
const processor = new LogProcessor();
processor.start().catch(error => {
  console.error('[LOG PROCESSOR] Fatal error:', error);
  process.exit(1);
});