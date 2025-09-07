import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
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

    // Check if invitation is for the current user
    if (invitation.email !== req.user!.email) {
      return res.status(403).json({ error: 'This invitation is not for you' });
    }

    // Check if already used
    if (invitation.usedAt) {
      return res.status(400).json({ error: 'Invitation has already been processed' });
    }

    // Delete the invitation (declined)
    await prisma.userInvitation.delete({
      where: { id: invitation.id },
    });

    return res.status(200).json({
      success: true,
      message: 'Invitation declined',
    });
  } catch (error) {
    console.error('Decline invitation error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
