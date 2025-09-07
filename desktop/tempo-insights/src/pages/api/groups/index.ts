import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient, GroupRole } from '@prisma/client';
import { z } from 'zod';
import { generateUniqueSlug } from '../../../lib/utils/slug';

const prisma = new PrismaClient();

const createGroupSchema = z.object({
  name: z.string().min(1).max(100),
  description: z.string().max(500).optional().nullable(),
  isPublic: z.boolean().default(true),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method === 'POST') {
    try {
      const { name, description, isPublic } = createGroupSchema.parse(req.body);

      // Generate unique slug
      const slug = await generateUniqueSlug(name, 'group');

      // Create group with creator as owner
      const group = await prisma.group.create({
        data: {
          name,
          slug,
          description: description || null,
          isPublic,
          members: {
            create: {
              userId: req.user!.id,
              role: GroupRole.OWNER,
            },
          },
        },
        include: {
          members: {
            include: {
              user: true,
            },
          },
          _count: {
            select: {
              members: true,
            },
          },
        },
      });

      return res.status(201).json({
        group: {
          ...group,
          memberCount: group._count.members,
          isOwner: true,
          isAdmin: true,
          isMember: true,
        },
      });
    } catch (error) {
      if (error instanceof z.ZodError) {
        return res.status(400).json({
          error: 'Invalid request data',
          details: error.issues,
        });
      }

      console.error('Create group error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  }

  if (req.method === 'GET') {
    try {
      // Get all groups the user is a member of
      const groups = await prisma.group.findMany({
        where: {
          members: {
            some: {
              userId: req.user!.id,
            },
          },
        },
        include: {
          members: {
            where: {
              userId: req.user!.id,
            },
          },
          _count: {
            select: {
              members: true,
            },
          },
        },
        orderBy: {
          createdAt: 'desc',
        },
      });

      // Get public groups the user is not a member of
      const publicGroups = await prisma.group.findMany({
        where: {
          isPublic: true,
          members: {
            none: {
              userId: req.user!.id,
            },
          },
        },
        include: {
          _count: {
            select: {
              members: true,
            },
          },
        },
        orderBy: {
          createdAt: 'desc',
        },
      });

      return res.status(200).json({
        myGroups: groups.map(group => ({
          ...group,
          memberCount: group._count.members,
          userRole: group.members[0]?.role || null,
        })),
        publicGroups: publicGroups.map(group => ({
          ...group,
          memberCount: group._count.members,
        })),
      });
    } catch (error) {
      console.error('List groups error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  }

  return res.status(405).json({ error: 'Method not allowed' });
});
