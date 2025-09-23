// /pages/api/users/me.ts
import { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { z } from 'zod';

const prisma = new PrismaClient();

// Schema for updating user profile
const updateProfileSchema = z.object({
  name: z.string().min(2).max(100).optional(),
  email: z.string().email().optional(),
  nextJumpNumber: z.number().int().min(1).optional(),
  homeDropzoneId: z.string().nullable().optional(),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  const user = req.user!; // User is guaranteed to exist after withAuth middleware

  if (req.method === 'GET') {
    try {
      // Fetch full user profile with counts
      const fullUser = await prisma.user.findUnique({
        where: { id: user.id },
        select: {
          id: true,
          email: true,
          name: true,
          slug: true,
          role: true,
          nextJumpNumber: true,
          homeDropzoneId: true,
          homeDropzone: {
            select: {
              id: true,
              name: true,
              slug: true,
            },
          },
          createdAt: true,
          _count: {
            select: {
              jumpLogs: true,
              groupMemberships: true,
              ownedDevices: true,
            },
          },
        },
      });

      if (!fullUser) {
        return res.status(404).json({ error: 'User not found' });
      }

      // Format response
      const userProfile = {
        ...fullUser,
        jumpCount: fullUser._count.jumpLogs,
        groupCount: fullUser._count.groupMemberships,
        deviceCount: fullUser._count.ownedDevices,
      };

      // Remove _count from response
      const { _count, ...userData } = userProfile;

      res.status(200).json({ user: userData });
    } catch (error) {
      console.error('Error fetching user profile:', error);
      res.status(500).json({ error: 'Failed to fetch profile' });
    }
  } else if (req.method === 'PATCH') {
    try {
      // Validate request body
      const validationResult = updateProfileSchema.safeParse(req.body);
      
      if (!validationResult.success) {
        return res.status(400).json({ 
          error: 'Invalid input', 
          details: validationResult.error.format() 
        });
      }

      const { name, email, nextJumpNumber, homeDropzoneId } = validationResult.data;

      // Build update data object
      const updateData: any = {};
      
      if (name !== undefined) updateData.name = name;
      if (email !== undefined) updateData.email = email;
      if (nextJumpNumber !== undefined) updateData.nextJumpNumber = nextJumpNumber;
      if (homeDropzoneId !== undefined) {
        // If empty string is provided, set to null
        updateData.homeDropzoneId = homeDropzoneId === '' ? null : homeDropzoneId;
      }

      // Check if email is being changed and ensure it's unique
      if (email && email !== user.email) {
        const existingUser = await prisma.user.findUnique({
          where: { email },
        });

        if (existingUser) {
          return res.status(400).json({ error: 'Email already in use' });
        }
      }

      // Verify dropzone exists if provided
      if (homeDropzoneId && homeDropzoneId !== '') {
        const dropzone = await prisma.dropzone.findUnique({
          where: { id: homeDropzoneId },
        });

        if (!dropzone) {
          return res.status(400).json({ error: 'Invalid dropzone selected' });
        }
      }

      // Update user
      const updatedUser = await prisma.user.update({
        where: { id: user.id },
        data: updateData,
        select: {
          id: true,
          email: true,
          name: true,
          slug: true,
          role: true,
          nextJumpNumber: true,
          homeDropzoneId: true,
          homeDropzone: {
            select: {
              id: true,
              name: true,
              slug: true,
            },
          },
          createdAt: true,
          _count: {
            select: {
              jumpLogs: true,
              groupMemberships: true,
              ownedDevices: true,
            },
          },
        },
      });

      // Format response
      const userProfile = {
        ...updatedUser,
        jumpCount: updatedUser._count.jumpLogs,
        groupCount: updatedUser._count.groupMemberships,
        deviceCount: updatedUser._count.ownedDevices,
      };

      // Remove _count from response
      const { _count, ...userData } = userProfile;

      res.status(200).json({ user: userData });
    } catch (error) {
      console.error('Error updating user profile:', error);
      res.status(500).json({ error: 'Failed to update profile' });
    }
  } else {
    res.setHeader('Allow', ['GET', 'PATCH']);
    res.status(405).json({ error: 'Method not allowed' });
  }
});