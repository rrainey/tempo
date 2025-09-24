// pages/api/jumps/import/confirm.ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { PrismaClient, DeviceState } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { z } from 'zod';

const prisma = new PrismaClient();
const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

// Validation schema for the request body
const confirmSchema = z.object({
  hash: z.string(),
  jumpNumber: z.number().int().positive().optional(),
  notes: z.string().nullable().optional(),
});

declare global {
  var importCache: Map<string, {
    buffer: Buffer;
    originalFileName: string;
    userId: string;
    timestamp: number;
  }>;
}

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Validate request body
    const body = confirmSchema.parse(req.body);

    // Retrieve cached data
    const cachedData = global.importCache?.get(body.hash);
    
    if (!cachedData) {
      return res.status(400).json({ 
        error: 'Import session expired. Please upload the file again.' 
      });
    }

    // Verify the cached data belongs to this user
    if (cachedData.userId !== req.user!.id) {
      return res.status(403).json({ error: 'Unauthorized' });
    }

    // Get or create a manual import device for this user
    let manualDevice = await prisma.device.findFirst({
      where: {
        ownerId: req.user!.id,
        name: 'Manual Import',
      },
    });

    if (!manualDevice) {
      manualDevice = await prisma.device.create({
        data: {
          bluetoothId: `manual-${req.user!.id}`,
          name: 'Manual Import',
          state: DeviceState.ACTIVE,
          ownerId: req.user!.id,
          lastSeen: new Date(),
        },
      });
    }

    // Upload to Supabase Storage - same pattern as bluetooth scanner
    const storagePath = `${req.user!.id}/${manualDevice.id}/${body.hash}.log`;
    console.log(`[IMPORT] Uploading to Supabase Storage: ${storagePath}`);
    
    const { data: uploadData, error: uploadError } = await supabase.storage
      .from('jump-logs')
      .upload(storagePath, cachedData.buffer, {
        contentType: 'application/octet-stream',
        upsert: false
      });
    
    if (uploadError) {
      throw new Error(`Failed to upload to storage: ${uploadError.message}`);
    }

    console.log(`[IMPORT] Successfully uploaded to storage`);
    
    // Get the storage URL
    const storageUrl = `${process.env.NEXT_PUBLIC_SUPABASE_URL}/storage/v1/object/jump-logs/${storagePath}`;

    // Handle jump number - either use provided or auto-increment
    let jumpNumber = body.jumpNumber;
    
    if (jumpNumber) {
      // If user provided a specific jump number, update nextJumpNumber if needed
      const currentUser = await prisma.user.findUnique({
        where: { id: req.user!.id },
        select: { nextJumpNumber: true },
      });
      
      await prisma.user.update({
        where: { id: req.user!.id },
        data: {
          nextJumpNumber: Math.max(jumpNumber + 1, currentUser?.nextJumpNumber || 1)
        }
      });
    } else {
      // Use and increment the user's nextJumpNumber
      const user = await prisma.user.update({
        where: { id: req.user!.id },
        data: {
          nextJumpNumber: { increment: 1 }
        },
        select: {
          nextJumpNumber: true
        }
      });
      jumpNumber = user.nextJumpNumber - 1;
    }

    // Create the jump log entry - minimal data like bluetooth scanner
    const jumpLog = await prisma.jumpLog.create({
      data: {
        hash: body.hash,
        jumpNumber: jumpNumber,
        storageUrl: storageUrl,
        storagePath: storagePath,
        fileSize: cachedData.buffer.length,
        mimeType: 'application/octet-stream',
        deviceId: manualDevice.id,
        userId: req.user!.id,
        offsets: {}, // Will be populated by analysis worker
        flags: {
          originalFileName: cachedData.originalFileName,
          uploadedAt: new Date().toISOString(),
          manualImport: true,
        },
        visibleToConnections: true,
        notes: body.notes || null,
        // Analysis fields left null - worker will populate
      },
    });

    console.log(`[IMPORT] Created JumpLog with ID ${jumpLog.id} for user ${req.user!.id}`);
    console.log(`[IMPORT]   - Jump number: ${jumpNumber}`);
    console.log(`[IMPORT]   - Storage path: ${storagePath}`);
    console.log(`[IMPORT]   - File size: ${cachedData.buffer.length} bytes`);

    // Clean up the cached data
    global.importCache.delete(body.hash);

    return res.status(200).json({
      message: 'Jump imported successfully',
      jumpId: jumpLog.id,
    });

  } catch (error) {
    console.error('[IMPORT] Error:', error);
    
    if (error instanceof z.ZodError) {
      return res.status(400).json({ error: 'Invalid request data', details: error });
    }
    
    return res.status(500).json({ 
      error: error instanceof Error ? error.message : 'Failed to save jump' 
    });
  }
});