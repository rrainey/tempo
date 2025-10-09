// pages/api/groups/[slug]/delete.ts

import { NextApiResponse } from 'next';
import { PrismaClient, GroupRole } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';

const prisma = new PrismaClient();

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'DELETE') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const { slug } = req.query;
  if (typeof slug !== 'string') {
    return res.status(400).json({ error: 'Invalid group slug' });
  }

  try {
    // Find the group
    const group = await prisma.group.findUnique({
      where: { slug },
      include: {
        members: {
          include: {
            user: {
              select: {
                id: true,
                name: true,
                slug: true,
              }
            }
          }
        }
      }
    });

    if (!group) {
      return res.status(404).json({ error: 'Group not found' });
    }

    // Check if user is an admin or owner
    const userMembership = group.members.find(m => m.userId === req.user!.id);
    if (!userMembership || (userMembership.role !== GroupRole.ADMIN && userMembership.role !== GroupRole.OWNER)) {
      return res.status(403).json({ error: 'Only group admins can delete the group' });
    }

    // Count total admins/owners
    const adminCount = group.members.filter(
      m => m.role === GroupRole.ADMIN || m.role === GroupRole.OWNER
    ).length;

    // If this is the last admin and there are other members, prevent deletion
    if (adminCount === 1 && group.members.length > 1) {
      return res.status(400).json({ 
        error: 'Cannot delete group',
        message: 'You are the last admin. Please promote another member to admin before deleting the group, or remove all other members first.',
        requiresAdminReassignment: true,
        members: group.members
          .filter(m => m.userId !== req.user!.id)
          .map(m => ({
            id: m.id,
            userId: m.userId,
            name: m.user.name,
            slug: m.user.slug,
            role: m.role,
          }))
      });
    }

    // Delete the group and all related data in a transaction
    await prisma.$transaction(async (tx) => {
      // Delete all group invitations
      await tx.userInvitation.deleteMany({
        where: { groupId: group.id }
      });

      // Delete all group memberships
      await tx.groupMember.deleteMany({
        where: { groupId: group.id }
      });

      // Delete the group itself
      await tx.group.delete({
        where: { id: group.id }
      });
    });

    res.status(200).json({
      success: true,
      message: 'Group deleted successfully'
    });

  } catch (error) {
    console.error('Error deleting group:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);