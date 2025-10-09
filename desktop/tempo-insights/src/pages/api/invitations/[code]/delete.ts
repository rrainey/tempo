// pages/api/invitations/[code]/delete.ts

import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';

const prisma = new PrismaClient();

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'DELETE') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const { code } = req.query;
  if (typeof code !== 'string') {
    return res.status(400).json({ error: 'Invalid invitation code' });
  }

  try {
    // Find the invitation
    const invitation = await prisma.userInvitation.findUnique({
      where: { code },
    });

    if (!invitation) {
      return res.status(404).json({ error: 'Invitation not found' });
    }

    // Only the creator can delete the invitation
    if (invitation.invitedById !== req.user.id) {
      return res.status(403).json({ error: 'You can only delete invitations you created' });
    }

    // Check if the invitation has already been used
    if (invitation.usedAt) {
      return res.status(400).json({ error: 'Cannot delete an invitation that has already been used' });
    }

    // Delete the invitation
    await prisma.userInvitation.delete({
      where: { code },
    });

    res.status(200).json({
      success: true,
      message: 'Invitation deleted successfully'
    });

  } catch (error) {
    console.error('Error deleting invitation:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);