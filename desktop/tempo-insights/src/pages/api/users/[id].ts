// Save as: src/pages/api/users/[id].ts

import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: userId } = req.query;
    
    if (!userId || typeof userId !== 'string') {
      return res.status(400).json({ error: 'Invalid user ID' });
    }

    // Check permissions
    const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';
    const isSelf = req.user!.id === userId;

    if (!isAdmin && !isSelf) {
      return res.status(403).json({ error: 'You do not have permission to view this user' });
    }

    // Get the user
    const user = await prisma.user.findUnique({
      where: { id: userId },
      select: {
        id: true,
        name: true,
        email: true,
        slug: true,
        role: true,
        nextJumpNumber: true,
        isProxy: true,
        createdAt: true,
        _count: {
          select: {
            jumpLogs: true,
            ownedDevices: true
          }
        }
      }
    });

    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }

    return res.status(200).json({
      user: {
        id: user.id,
        name: user.name,
        email: user.email,
        slug: user.slug,
        role: user.role,
        nextJumpNumber: user.nextJumpNumber,
        isProxy: user.isProxy,
        jumpCount: user._count.jumpLogs,
        deviceCount: user._count.ownedDevices,
        createdAt: user.createdAt
      }
    });

  } catch (error) {
    console.error('Get user error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});