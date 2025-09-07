import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  const { slug } = req.query;

  if (typeof slug !== 'string') {
    return res.status(400).json({ error: 'Invalid slug' });
  }

  if (req.method === 'GET') {
    try {
      // Get group with members
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
                  email: true,
                },
              },
            },
            orderBy: [
              { role: 'desc' }, // OWNER first, then ADMIN, then MEMBER
              { joinedAt: 'asc' },
            ],
          },
          _count: {
            select: {
              members: true,
            },
          },
        },
      });

      if (!group) {
        return res.status(404).json({ error: 'Group not found' });
      }

      // Check if user is a member
      const userMembership = group.members.find(
        member => member.userId === req.user!.id
      );

      // Format response
      const groupResponse = {
        id: group.id,
        name: group.name,
        slug: group.slug,
        description: group.description,
        isPublic: group.isPublic,
        createdAt: group.createdAt,
        memberCount: group._count.members,
        userRole: userMembership?.role || null,
        isMember: !!userMembership,
        members: group.members.map(member => ({
          id: member.user.id,
          name: member.user.name,
          slug: member.user.slug,
          email: member.user.email,
          role: member.role,
          joinedAt: member.joinedAt,
        })),
      };

      return res.status(200).json({ group: groupResponse });
    } catch (error) {
      console.error('Get group error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  }

  return res.status(405).json({ error: 'Method not allowed' });
});
