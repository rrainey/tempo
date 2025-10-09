// pages/api/invitations/pending.ts

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
    // Get all unused invitations created by the current user
    const invitations = await prisma.userInvitation.findMany({
      where: {
        invitedById: req.user.id,
        usedAt: null, // Only unused invitations
      },
      include: {
        group: {
          select: {
            id: true,
            name: true,
            slug: true,
          }
        }
      },
      orderBy: {
        createdAt: 'desc'
      }
    });

    // Format the response
    const formattedInvitations = invitations.map(inv => ({
      id: inv.id,
      code: inv.code,
      email: inv.email,
      group: inv.group,
      groupRole: inv.groupRole,
      userRole: inv.userRole,
      expiresAt: inv.expiresAt.toISOString(),
      createdAt: inv.createdAt.toISOString(),
    }));

    res.status(200).json({
      invitations: formattedInvitations
    });

  } catch (error) {
    console.error('Error fetching pending invitations:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);