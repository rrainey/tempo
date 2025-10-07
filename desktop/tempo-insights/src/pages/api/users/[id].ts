// Save as: src/pages/api/users/[id].ts

import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: targetId } = req.query;
    
    if (!targetId || typeof targetId !== 'string') {
      return res.status(400).json({ error: 'Invalid user ID' });
    }

    const currentUserId = req.user!.id;
    const isAdmin = req.user!.role === 'ADMIN' || req.user!.role === 'SUPER_ADMIN';
    const isSelf = currentUserId === targetId;

    // First try to get user by ID
    let user = await prisma.user.findUnique({
      where: { id: targetId },
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

    // If not found by ID, try by slug (for public profile viewing)
    if (!user && targetId.match(/^[a-z0-9-]+$/)) {
      user = await prisma.user.findUnique({
        where: { slug: targetId },
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
    }

    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }

    // Update isSelf check with actual user ID
    const actualIsSelf = currentUserId === user.id;

    // Check connection status if not viewing self
    let connectionStatus = 'self';
    if (!actualIsSelf) {
      // Check if connected
      const connection = await prisma.connection.findFirst({
        where: {
          OR: [
            { userId1: currentUserId, userId2: user.id },
            { userId1: user.id, userId2: currentUserId }
          ]
        }
      });

      if (connection) {
        connectionStatus = 'connected';
      } else {
        // Check for pending requests
        const [sentRequest, receivedRequest] = await Promise.all([
          prisma.connectionRequest.findFirst({
            where: {
              fromUserId: currentUserId,
              toUserId: user.id,
              status: 'PENDING'
            }
          }),
          prisma.connectionRequest.findFirst({
            where: {
              fromUserId: user.id,
              toUserId: currentUserId,
              status: 'PENDING'
            }
          })
        ]);

        if (sentRequest) {
          connectionStatus = 'request_sent';
        } else if (receivedRequest) {
          connectionStatus = 'request_received';
        } else {
          connectionStatus = 'none';
        }
      }
    }

    // For non-admin users viewing others, show limited info unless connected
    if (!isAdmin && !actualIsSelf) {
      // Public profile view - limited info
      return res.status(200).json({
        user: {
          id: user.id,
          name: user.name,
          slug: user.slug,
          isProxy: user.isProxy,
          jumpCount: user._count.jumpLogs,
          connectionStatus
        }
      });
    }

    // Full profile for self or admin
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
        createdAt: user.createdAt,
        connectionStatus
      }
    });

  } catch (error) {
    console.error('Get user error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});