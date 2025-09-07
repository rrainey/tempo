import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';
import { generateUniqueSlug } from '../../../lib/utils/slug';

const prisma = new PrismaClient();

const updateProfileSchema = z.object({
  name: z.string().min(2).optional(),
  email: z.string().email().optional(),
  photo: z.object({
    mimeType: z.string(),
    data: z.string(), // Base64 encoded
  }).optional(),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method === 'GET') {
    // Get user profile with additional stats
    const userWithStats = await prisma.user.findUnique({
      where: { id: req.user!.id },
      include: {
        _count: {
          select: {
            jumpLogs: true,
            groupMemberships: true,
            ownedDevices: true,
          },
        },
      },
    });

    if (!userWithStats) {
      return res.status(404).json({ error: 'User not found' });
    }

    // Remove password from response
    const { password, ...userProfile } = userWithStats;

    return res.status(200).json({
      user: {
        ...userProfile,
        jumpCount: userProfile._count.jumpLogs,
        groupCount: userProfile._count.groupMemberships,
        deviceCount: userProfile._count.ownedDevices,
      },
    });
  }

  if (req.method === 'PATCH') {
    try {
      const updates = updateProfileSchema.parse(req.body);
      const updateData: any = {};

      // Handle name update and slug regeneration
      if (updates.name && updates.name !== req.user!.name) {
        updateData.name = updates.name;
        updateData.slug = await generateUniqueSlug(updates.name, 'user', req.user!.id);
      }

      // Handle email update
      if (updates.email && updates.email !== req.user!.email) {
        // Check if email is already taken
        const existingUser = await prisma.user.findUnique({
          where: { email: updates.email.toLowerCase() },
        });

        if (existingUser && existingUser.id !== req.user!.id) {
          return res.status(409).json({ error: 'Email already in use' });
        }

        updateData.email = updates.email.toLowerCase();
      }

      // Handle photo update
      if (updates.photo) {
        // In a real app, you'd save this to storage
        // For MVP, we'll store it in a new field (needs migration)
        console.log('Photo update received:', updates.photo.mimeType);
        // TODO: Implement photo storage
      }

      // Update user
      const updatedUser = await prisma.user.update({
        where: { id: req.user!.id },
        data: updateData,
        include: {
          _count: {
            select: {
              jumpLogs: true,
              groupMemberships: true,
              ownedDevices: true,
            },
          },
        },
      });

      const { password, ...userProfile } = updatedUser;

      return res.status(200).json({
        user: {
          ...userProfile,
          jumpCount: userProfile._count.jumpLogs,
          groupCount: userProfile._count.groupMemberships,
          deviceCount: userProfile._count.ownedDevices,
        },
      });
    } catch (error) {
      if (error instanceof z.ZodError) {
        return res.status(400).json({
          error: 'Invalid request data',
          details: error.issues
        });
      }

      console.error('Profile update error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  }

  return res.status(405).json({ error: 'Method not allowed' });
});
