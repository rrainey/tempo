// pages/api/internal/jumps/create.ts
import type { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { z } from 'zod';
import crypto from 'crypto';

const prisma = new PrismaClient();

// Initialize Supabase client with service key for server-side operations
const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

const createJumpLogSchema = z.object({
  deviceId: z.string(),
  userId: z.string(),
  rawLogBase64: z.string(), // Base64 encoded binary data
  fileName: z.string(),
  timestamp: z.string().datetime().optional(),
});

// Verify internal API token
function verifyInternalToken(req: NextApiRequest): boolean {
  const token = req.headers['x-internal-token'];
  return token === process.env.INTERNAL_API_TOKEN;
}

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  // Verify internal token
  if (!verifyInternalToken(req)) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  try {
    const { deviceId, userId, rawLogBase64, fileName, timestamp } = createJumpLogSchema.parse(req.body);

    // Convert base64 to buffer
    const rawLogBuffer = Buffer.from(rawLogBase64, 'base64');

    // Calculate SHA-256 hash of the raw log
    const hash = crypto.createHash('sha256').update(rawLogBuffer).digest('hex');

    // Check if this log already exists (by hash)
    const existingLog = await prisma.jumpLog.findUnique({
      where: { hash },
    });

    if (existingLog) {
      return res.status(200).json({
        jumpLog: existingLog,
        message: 'Jump log already exists',
        isNew: false,
      });
    }

    // Generate unique ID for this jump log
    const jumpId = crypto.randomUUID();

    // Create storage path
    const storagePath = `users/${userId}/jumps/${jumpId}/${fileName}`;

    // Upload to Supabase Storage
    const { data: uploadData, error: uploadError } = await supabase.storage
      .from('jump-logs')
      .upload(storagePath, rawLogBuffer, {
        contentType: 'application/octet-stream',
        upsert: false, // Prevent accidental overwrites
      });

    if (uploadError) {
      console.error('Storage upload error:', uploadError);
      return res.status(500).json({ 
        error: 'Failed to upload jump log to storage',
        details: uploadError.message 
      });
    }

    // Get the public URL for the uploaded file
    const { data: urlData } = supabase.storage
      .from('jump-logs')
      .getPublicUrl(storagePath);

    // Get the user's current jump number and increment it
    const user = await prisma.user.findUnique({
      where: { id: userId },
      select: { nextJumpNumber: true }
    });

    const jumpNumber = user?.nextJumpNumber || 1;

    // Create jump log record in database
    const jumpLog = await prisma.jumpLog.create({
      data: {
        id: jumpId,
        deviceId,
        userId,
        jumpNumber,
        hash,
        storageUrl: urlData.publicUrl,
        storagePath: storagePath,
        fileSize: rawLogBuffer.length,
        mimeType: 'application/octet-stream',
        offsets: {}, // Will be populated by analysis worker
        flags: {
          fileName,
          uploadTimestamp: timestamp || new Date().toISOString(),
        },
      },
      include: {
        device: true,
        user: {
          select: {
            id: true,
            name: true,
            email: true,
          },
        },
      },
    });

    // Update user's next jump number
    await prisma.user.update({
      where: { id: userId },
      data: { nextJumpNumber: jumpNumber + 1 }
    });

    console.log(`[JUMP LOG] Created new jump log ${jumpLog.id} for user ${jumpLog.user.name} from device ${jumpLog.device.name}`);
    console.log(`[JUMP LOG] Stored at: ${storagePath} (${rawLogBuffer.length} bytes)`);

    return res.status(201).json({
      jumpLog: {
        id: jumpLog.id,
        jumpNumber: jumpLog.jumpNumber,
        hash: jumpLog.hash,
        deviceId: jumpLog.deviceId,
        userId: jumpLog.userId,
        storageUrl: jumpLog.storageUrl,
        storagePath: jumpLog.storagePath,
        fileSize: jumpLog.fileSize,
        createdAt: jumpLog.createdAt,
      },
      message: 'Jump log created successfully',
      isNew: true,
    });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid request data',
        details: error.issues,
      });
    }

    console.error('Create jump log error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
}

// For Next.js API routes handling file uploads
export const config = {
  api: {
    bodyParser: {
      sizeLimit: '20mb', // Increase limit to handle base64 encoded files up to ~15MB raw
    },
  },
};