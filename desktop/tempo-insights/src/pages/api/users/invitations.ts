import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method === 'GET') {
    try {
      // Get pending invitations for the user
      const invitations = await prisma.userInvitation.findMany({
        where: {
          email: req.user!.email,
          usedAt: null,
          expiresAt: {
            gt: new Date(),
          },
        },
        include: {
          group: true,
          invitedBy: {
            select: {
              name: true,
              email: true,
            },
          },
        },
        orderBy: {
          createdAt: 'desc',
        },
      });

      return res.status(200).json({
        invitations: invitations.map(inv => ({
          id: inv.id,
          code: inv.code,
          group: {
            id: inv.group?.id,
            name: inv.group?.name,
            slug: inv.group?.slug,
          },
          groupRole: inv.groupRole,
          invitedBy: inv.invitedBy.name,
          expiresAt: inv.expiresAt,
          createdAt: inv.createdAt,
        })),
      });
    } catch (error) {
      console.error('Get invitations error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  }

  return res.status(405).json({ error: 'Method not allowed' });
});
