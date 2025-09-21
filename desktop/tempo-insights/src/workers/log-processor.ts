import { PrismaClient } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { config } from 'dotenv';
import { LogParser } from '../lib/analysis/log-parser';
import { EventDetector } from '../lib/analysis/event-detector';

// Load environment variables
config({ path: '.env' });

const prisma = new PrismaClient();

const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

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
      // Query for pending logs (Task 65)
      const pendingLogs = await this.getQueuedLogs();
      
      if (pendingLogs.length > 0) {
        console.log(`[LOG PROCESSOR] Found ${pendingLogs.length} pending log(s) to process`);
        
        // Process each log
        for (const jumpLog of pendingLogs) {
          await this.processJumpLog(jumpLog);
        }
      } else {
        console.log(`[LOG PROCESSOR] No pending logs to process`);
      }
      
      // Task 73: Check for formation grouping after processing logs
      await this.checkFormationGrouping();
      
      const duration = Date.now() - startTime;
      console.log(`[LOG PROCESSOR] Cycle ${this.cycleCount} completed in ${duration}ms`);
      
    } catch (error) {
      console.error(`[LOG PROCESSOR] Error in cycle ${this.cycleCount}:`, error);
    }
    
    console.log(`[LOG PROCESSOR] Cycle end\n`);
  }

  private async getPendingLogCount(): Promise<number> {
    const count = await prisma.jumpLog.count({
      where: {
        initialAnalysisTimestamp: null
      }
    });
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
    console.log(`[LOG PROCESSOR]   - Size: ${jumpLog.fileSize} bytes`);
    
    try {

      // Download the log file from Supabase Storage
      const { data: fileData, error: downloadError } = await supabase.storage
        .from('jump-logs')
        .download(jumpLog.storagePath);
      
      if (downloadError) {
        throw new Error(`Failed to download log: ${downloadError.message}`);
      }
      
      // Convert blob to buffer
      const logBuffer = Buffer.from(await fileData.arrayBuffer());
      
      // Verify size matches
      if (logBuffer.length !== jumpLog.fileSize) {
        throw new Error(`File size mismatch: expected ${jumpLog.fileSize}, got ${logBuffer.length}`);
      }

      const validation = LogParser.validateLog(logBuffer);
      if (!validation.isValid) {
        throw new Error(`Invalid log: ${validation.message}`);
      }
      
      const parsedData = LogParser.parseLog(logBuffer);
      console.log(`[LOG PROCESSOR]   - Parsed: ${parsedData.altitude.length} altitude points`);
      console.log(`[LOG PROCESSOR]   - Duration: ${parsedData.duration}s`);
      console.log(`[LOG PROCESSOR]   - GPS: ${parsedData.hasGPS ? 'Yes' : 'No'}`);
      
      // Task 67: Detect events
      const events = EventDetector.analyzeJump(parsedData);
      console.log(`[LOG PROCESSOR]   - Events detected:`);
      
      if (events.exitOffsetSec !== undefined) {
        console.log(`[LOG PROCESSOR]     • Exit: ${events.exitOffsetSec.toFixed(1)}s at ${events.exitAltitudeFt || 'unknown'}ft`);
      }
      if (events.deploymentOffsetSec !== undefined) {
        console.log(`[LOG PROCESSOR]     • Deploy: ${events.deploymentOffsetSec.toFixed(1)}s at ${events.deployAltitudeFt || 'unknown'}ft`);
      }
      if (events.landingOffsetSec !== undefined) {
        console.log(`[LOG PROCESSOR]     • Landing: ${events.landingOffsetSec.toFixed(1)}s`);
      }
      
      // Task 70: Calculate exit timestamp and GPS location
      let exitTimestampUTC: Date | undefined;
      let exitLatitude: number | undefined;
      let exitLongitude: number | undefined;
      
      if (events.exitOffsetSec !== undefined) {
        // Calculate actual exit time
        exitTimestampUTC = new Date(parsedData.startTime.getTime() + events.exitOffsetSec * 1000);
        console.log(`[LOG PROCESSOR]     • Exit time: ${exitTimestampUTC.toISOString()}`);
        
        // Find GPS coordinates at exit if available
        if (events.exitOffsetSec !== undefined && parsedData.hasGPS && parsedData.gps.length > 0) {
          // Find the GPS point closest to exit time
          const exitGPS = parsedData.gps.find(p => 
            Math.abs(p.timestamp - events.exitOffsetSec!) < 0.5
          );  
          
          if (exitGPS) {
            exitLatitude = exitGPS.latitude;
            exitLongitude = exitGPS.longitude;
            //console.log(`[LOG PROCESSOR]     • Exit GPS: ${exitLatitude.toFixed(6)}, ${exitLongitude.toFixed(6)}`);
          }
        }

        if (events.exitLatitude !== undefined && events.exitLongitude !== undefined) {
          exitLatitude = events.exitLatitude;
          exitLongitude = events.exitLongitude;
          console.log(`[LOG PROCESSOR]     • Exit GPS: ${exitLatitude.toFixed(6)}, ${exitLongitude.toFixed(6)}`);
        }
      }
      
      // Task 71: Calculate freefall metrics
      let freefallTimeSec: number | undefined;
      let avgFallRateMph: number | undefined;
      
      if (events.exitOffsetSec !== undefined && events.deploymentOffsetSec !== undefined) {
        // Calculate freefall time
        freefallTimeSec = events.deploymentOffsetSec - events.exitOffsetSec;
        console.log(`[LOG PROCESSOR]     • Freefall time: ${freefallTimeSec.toFixed(1)}s`);
        
        // Calculate average fall rate
        if (events.exitAltitudeFt && events.deployAltitudeFt) {
          const altitudeLost = events.exitAltitudeFt - events.deployAltitudeFt;
          const avgFallRateFps = altitudeLost / freefallTimeSec; // feet per second
          avgFallRateMph = avgFallRateFps * 3600 / 5280; // Convert to mph
          console.log(`[LOG PROCESSOR]     • Avg fall rate: ${avgFallRateMph.toFixed(1)} mph`);
        }
      }
      
      // Update the jump log with detected events
      await prisma.jumpLog.update({
        where: { id: jumpLog.id },
        data: {
          initialAnalysisTimestamp: new Date(),
          initialAnalysisMessage: events.exitOffsetSec ? null : 'No exit detected',
          exitOffsetSec: events.exitOffsetSec,
          deploymentOffsetSec: events.deploymentOffsetSec,
          landingOffsetSec: events.landingOffsetSec,
          exitAltitudeFt: events.exitAltitudeFt,
          deployAltitudeFt: events.deployAltitudeFt,
          exitTimestampUTC: exitTimestampUTC,
          exitLatitude: exitLatitude,
          exitLongitude: exitLongitude,
          freefallTimeSec: freefallTimeSec,
          avgFallRateMph: avgFallRateMph,
          // Store detailed offsets in JSON
           offsets: {
            dataPoints: parsedData.altitude.length,
            duration: parsedData.duration,
            sampleRate: parsedData.sampleRate,
            exitOffset: events.exitOffsetSec,
            deployOffset: events.deploymentOffsetSec,
            landingOffset: events.landingOffsetSec,
            peakAcceleration: events.peakAcceleration,
            logVersion: parsedData.logVersion
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

  private async checkFormationGrouping() {
    // Task 73: Find logs with exit times within ±120 seconds
    const FORMATION_WINDOW_SEC = 120; // ±2 minutes
    
    console.log('[LOG PROCESSOR] Checking for formation grouping...');
    
    try {
      // Get recently analyzed logs that have exit times and aren't already in formations
      // Task 74: Only include logs that are visible to connections
      const recentLogs = await prisma.jumpLog.findMany({
        where: {
          exitTimestampUTC: {
            not: null
          },
          formationParticipant: null, // Not already in a formation
          visibleToConnections: true  // Respect visibility settings
        },
        orderBy: {
          exitTimestampUTC: 'desc'
        },
        take: 50, // Check last 50 jumps
        include: {
          user: true,
          device: true
        }
      });
      
      if (recentLogs.length < 2) {
        console.log('[LOG PROCESSOR] Not enough ungrouped jumps to form formations');
        return;
      }
      
      // Group logs by proximity in exit time
      const groups: any[][] = [];
      const used = new Set<string>();
      
      for (let i = 0; i < recentLogs.length; i++) {
        if (used.has(recentLogs[i].id)) continue;
        
        const group = [recentLogs[i]];
        used.add(recentLogs[i].id);
        
        // Find all other logs within the time window
        for (let j = i + 1; j < recentLogs.length; j++) {
          if (used.has(recentLogs[j].id)) continue;
          
          const timeDiff = Math.abs(
            recentLogs[i].exitTimestampUTC!.getTime() - 
            recentLogs[j].exitTimestampUTC!.getTime()
          ) / 1000; // Convert to seconds
          
          if (timeDiff <= FORMATION_WINDOW_SEC) {
            group.push(recentLogs[j]);
            used.add(recentLogs[j].id);
          }
        }
        
        // Only create formations with 2+ jumpers
        if (group.length >= 2) {
          groups.push(group);
        }
      }
      
      // Create formations for each group
      for (const group of groups) {
        console.log(`[LOG PROCESSOR] Found potential formation with ${group.length} jumpers`);
        
        // Use the earliest exit time as the formation jump time
        const jumpTime = new Date(Math.min(...group.map(log => log.exitTimestampUTC!.getTime())));
        
        // Generate formation name based on date and participant count
        const dateStr = jumpTime.toISOString().split('T')[0];
        const formationName = `${group.length}-way ${dateStr}`;
        
        // Get average exit altitude
        const avgAltitude = Math.round(
          group.reduce((sum, log) => sum + (log.exitAltitudeFt || 0), 0) / group.length
        );
        
        // Create the formation
        const formation = await prisma.formationSkydive.create({
          data: {
            name: formationName,
            jumpTime: jumpTime,
            altitude: avgAltitude > 0 ? avgAltitude : null,
            notes: `Auto-detected formation of ${group.length} jumpers`,
            isPublic: false, // Default to private
            participants: {
              create: group.map((log, index) => ({
                position: index + 1,
                userId: log.userId,
                jumpLogId: log.id
              }))
            }
          },
          include: {
            participants: {
              include: {
                user: true
              }
            }
          }
        });
        
        console.log(`[LOG PROCESSOR] Created formation ${formation.id}: ${formationName}`);
        formation.participants.forEach(p => {
          console.log(`[LOG PROCESSOR]   - Position ${p.position}: ${p.user.name || p.user.email}`);
        });
      }
      
      if (groups.length === 0) {
        console.log('[LOG PROCESSOR] No new formations detected');
      } else {
        console.log(`[LOG PROCESSOR] Created ${groups.length} formation(s)`);
      }
      
    } catch (error) {
      console.error('[LOG PROCESSOR] Error checking formations:', error);
    }
  }
}

// Start the processor
const processor = new LogProcessor();
processor.start().catch(error => {
  console.error('[LOG PROCESSOR] Fatal error:', error);
  process.exit(1);
});