// pages/api/groups/[slug]/members/[memberId]/promote.ts

import { NextApiResponse } from 'next';
import { PrismaClient, GroupRole } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../../../../lib/auth/middleware';

const prisma = new PrismaClient();

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const { slug, memberId } = req.query;
  if (typeof slug !== 'string' || typeof memberId !== 'string') {
    return res.status(400).json({ error: 'Invalid parameters' });
  }

  try {
    // Find the group
    const group = await prisma.group.findUnique({
      where: { slug },
      include: {
        members: true
      }
    });

    if (!group) {
      return res.status(404).json({ error: 'Group not found' });
    }

    // Check if requesting user is an admin or owner
    const requesterMembership = group.members.find(m => m.userId === req.user!.id);
    if (!requesterMembership || (requesterMembership.role !== GroupRole.ADMIN && requesterMembership.role !== GroupRole.OWNER)) {
      return res.status(403).json({ error: 'Only group admins can promote members' });
    }

    // Find the member to promote
    const targetMember = group.members.find(m => m.id === memberId);
    if (!targetMember) {
      return res.status(404).json({ error: 'Member not found' });
    }

    // Check if already an admin or owner
    if (targetMember.role === GroupRole.ADMIN || targetMember.role === GroupRole.OWNER) {
      return res.status(400).json({ error: 'Member is already an admin' });
    }

    // Promote to admin
    await prisma.groupMember.update({
      where: { id: memberId },
      data: { role: GroupRole.ADMIN }
    });

    res.status(200).json({
      success: true,
      message: 'Member promoted to admin successfully'
    });

  } catch (error) {
    console.error('Error promoting member:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);