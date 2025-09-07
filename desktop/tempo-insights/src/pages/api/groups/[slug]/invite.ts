import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient, GroupRole, UserRole } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const inviteSchema = z.object({
  email: z.string().email(),
  groupRole: z.enum(['MEMBER', 'ADMIN']).default('MEMBER'),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { slug } = req.query;

  if (typeof slug !== 'string') {
    return res.status(400).json({ error: 'Invalid slug' });
  }

  try {
    const { email, groupRole } = inviteSchema.parse(req.body);

    // Get the group and check user's role
    const group = await prisma.group.findUnique({
      where: { slug },
      include: {
        members: {
          where: {
            userId: req.user!.id,
          },
        },
      },
    });

    if (!group) {
      return res.status(404).json({ error: 'Group not found' });
    }

    // Check if user is admin or owner
    const userMembership = group.members[0];
    if (!userMembership || (userMembership.role !== 'ADMIN' && userMembership.role !== 'OWNER')) {
      return res.status(403).json({ error: 'Only group admins can invite members' });
    }

    // Find the user to invite
    const invitedUser = await prisma.user.findUnique({
      where: { email: email.toLowerCase() },
    });

    if (!invitedUser) {
      return res.status(404).json({ error: 'User not found' });
    }

    // Check if user is already a member
    const existingMembership = await prisma.groupMember.findUnique({
      where: {
        userId_groupId: {
          userId: invitedUser.id,
          groupId: group.id,
        },
      },
    });

    if (existingMembership) {
      return res.status(400).json({ error: 'User is already a member of this group' });
    }

    // Check if invitation already exists
    const existingInvitation = await prisma.userInvitation.findFirst({
      where: {
        email: invitedUser.email,
        groupId: group.id,
        usedAt: null,
        expiresAt: {
          gt: new Date(),
        },
      },
    });

    if (existingInvitation) {
      return res.status(400).json({ error: 'An invitation has already been sent to this user' });
    }

    // Create invitation
    const invitation = await prisma.userInvitation.create({
      data: {
        email: invitedUser.email,
        code: generateInviteCode(),
        groupId: group.id,
        groupRole: groupRole as GroupRole,
        userRole: UserRole.USER,
        invitedById: req.user!.id,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000), // 7 days
      },
      include: {
        group: true,
        invitedBy: {
          select: {
            name: true,
          },
        },
      },
    });

    return res.status(200).json({
      success: true,
      message: `Invitation sent to ${invitedUser.email}`,
      invitation: {
        id: invitation.id,
        email: invitation.email,
        groupName: invitation.group?.name,
        invitedBy: invitation.invitedBy.name,
        expiresAt: invitation.expiresAt,
      },
    });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid request data',
        details: error.issues,
      });
    }

    console.error('Invite to group error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});

function generateInviteCode(): string {
  return Math.random().toString(36).substring(2, 15) + Math.random().toString(36).substring(2, 15);
}
