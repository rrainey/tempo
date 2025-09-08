import type { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';
import crypto from 'crypto';

const prisma = new PrismaClient();

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

    // Create new jump log
    const jumpLog = await prisma.jumpLog.create({
      data: {
        deviceId,
        userId,
        hash,
        rawLog: rawLogBuffer,
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

    console.log(`[JUMP LOG] Created new jump log ${jumpLog.id} for user ${jumpLog.user.name} from device ${jumpLog.device.name}`);

    return res.status(201).json({
      jumpLog: {
        id: jumpLog.id,
        hash: jumpLog.hash,
        deviceId: jumpLog.deviceId,
        userId: jumpLog.userId,
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
