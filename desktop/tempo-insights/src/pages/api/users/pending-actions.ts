// src/pages/api/users/pending-actions.ts
import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const userId = req.user!.id;

    // Get all pending items in parallel
    const [connectionRequests, groupInvitations] = await Promise.all([
      // Connection requests received
      prisma.connectionRequest.findMany({
        where: {
          toUserId: userId,
          status: 'PENDING'
        },
        include: {
          fromUser: {
            select: { id: true, name: true, slug: true }
          }
        },
        orderBy: { createdAt: 'desc' }
      }),
      
      // Group invitations (if user has been invited by email)
      prisma.userInvitation.findMany({
        where: {
          email: req.user!.email,
          usedAt: null,
          expiresAt: { gt: new Date() },
          groupId: { not: null }
        },
        include: {
          group: {
            select: { id: true, name: true, slug: true }
          },
          invitedBy: {
            select: { id: true, name: true, slug: true }
          }
        },
        orderBy: { createdAt: 'desc' }
      })
    ]);

    // Format the response
    const pendingActions = {
      connectionRequests: connectionRequests.map(req => ({
        id: req.id,
        type: 'connection_request',
        from: req.fromUser,
        message: req.message,
        createdAt: req.createdAt
      })),
      groupInvitations: groupInvitations.map(inv => ({
        id: inv.id,
        type: 'group_invitation',
        group: inv.group,
        invitedBy: inv.invitedBy,
        expiresAt: inv.expiresAt,
        createdAt: inv.createdAt
      })),
      total: connectionRequests.length + groupInvitations.length
    };

    return res.status(200).json(pendingActions);

  } catch (error) {
    console.error('Pending actions error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});