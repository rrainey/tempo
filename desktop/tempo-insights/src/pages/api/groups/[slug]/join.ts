import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient, GroupRole } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { slug } = req.query;

  if (typeof slug !== 'string') {
    return res.status(400).json({ error: 'Invalid slug' });
  }

  try {
    // Get the group
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

    // Check if group is public
    if (!group.isPublic) {
      return res.status(403).json({ error: 'Cannot join private group without invitation' });
    }

    // Check if already a member
    if (group.members.length > 0) {
      return res.status(400).json({ error: 'Already a member of this group' });
    }

    // Add user to group
    const membership = await prisma.groupMember.create({
      data: {
        userId: req.user!.id,
        groupId: group.id,
        role: GroupRole.MEMBER,
      },
      include: {
        group: {
          include: {
            _count: {
              select: {
                members: true,
              },
            },
          },
        },
      },
    });

    return res.status(200).json({
      success: true,
      message: 'Successfully joined group',
      group: {
        ...membership.group,
        memberCount: membership.group._count.members,
        userRole: membership.role,
      },
    });
  } catch (error) {
    console.error('Join group error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
