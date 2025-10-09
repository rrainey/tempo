// pages/api/export/size.ts

import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  try {
    const userId = req.user.id;

    // Get all jump logs for the user
    const jumpLogs = await prisma.jumpLog.findMany({
      where: { userId },
      select: {
        id: true,
        fileSize: true,
        jumpNumber: true,
      }
    });

    // Calculate total size of raw log files
    const totalRawLogSize = jumpLogs.reduce((sum, log) => sum + log.fileSize, 0);
    
    // Estimate JSON metadata size (rough estimate: ~2KB per jump + 50KB base)
    const estimatedMetadataSize = (jumpLogs.length * 2048) + 51200;
    
    // Total estimated size (raw logs + metadata)
    const estimatedTotalSize = totalRawLogSize + estimatedMetadataSize;

    // Count of items
    const jumpCount = jumpLogs.length;

    res.status(200).json({
      jumpCount,
      totalRawLogSize,
      estimatedMetadataSize,
      estimatedTotalSize,
      estimatedTotalSizeMB: (estimatedTotalSize / (1024 * 1024)).toFixed(2),
    });
  } catch (error) {
    console.error('Error estimating export size:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);