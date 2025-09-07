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
      include: {
        group: true,
      },
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
      return res.status(400).json({ error: 'Invitation has already been used' });
    }

    // Check if expired
    if (invitation.expiresAt < new Date()) {
      return res.status(400).json({ error: 'Invitation has expired' });
    }

    // Check if user is already a member
    const existingMembership = await prisma.groupMember.findUnique({
      where: {
        userId_groupId: {
          userId: req.user!.id,
          groupId: invitation.groupId!,
        },
      },
    });

    if (existingMembership) {
      // Mark invitation as used anyway
      await prisma.userInvitation.update({
        where: { id: invitation.id },
        data: {
          usedAt: new Date(),
          usedById: req.user!.id,
        },
      });

      return res.status(400).json({ error: 'You are already a member of this group' });
    }

    // Accept invitation - create membership and mark as used
    const [membership, _] = await prisma.$transaction([
      prisma.groupMember.create({
        data: {
          userId: req.user!.id,
          groupId: invitation.groupId!,
          role: invitation.groupRole!,
        },
        include: {
          group: true,
        },
      }),
      prisma.userInvitation.update({
        where: { id: invitation.id },
        data: {
          usedAt: new Date(),
          usedById: req.user!.id,
        },
      }),
    ]);

    return res.status(200).json({
      success: true,
      message: `Successfully joined ${membership.group.name}`,
      group: {
        id: membership.group.id,
        name: membership.group.name,
        slug: membership.group.slug,
      },
    });
  } catch (error) {
    console.error('Accept invitation error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
