// src/pages/api/connections/request.ts
import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const requestSchema = z.object({
  toUserId: z.string(),
  message: z.string().max(500).optional()
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const validation = requestSchema.safeParse(req.body);
    if (!validation.success) {
      return res.status(400).json({ error: 'Invalid input', details: validation.error });
    }

    const { toUserId, message } = validation.data;
    const fromUserId = req.user!.id;

    // Can't send request to yourself
    if (fromUserId === toUserId) {
      return res.status(400).json({ error: 'Cannot send connection request to yourself' });
    }

    // Check if target user exists
    const targetUser = await prisma.user.findUnique({
      where: { id: toUserId }
    });

    if (!targetUser) {
      return res.status(404).json({ error: 'User not found' });
    }

    // Check if already connected
    const existingConnection = await prisma.connection.findFirst({
      where: {
        OR: [
          { userId1: fromUserId, userId2: toUserId },
          { userId1: toUserId, userId2: fromUserId }
        ]
      }
    });

    if (existingConnection) {
      return res.status(400).json({ error: 'Already connected with this user' });
    }

    // Check if request already exists
    const existingRequest = await prisma.connectionRequest.findUnique({
      where: { fromUserId_toUserId: { fromUserId, toUserId } }
    });

    if (existingRequest) {
      if (existingRequest.status === 'PENDING') {
        return res.status(400).json({ error: 'Connection request already pending' });
      } else if (existingRequest.status === 'DECLINED') {
        // Allow resending if previously declined
        const updatedRequest = await prisma.connectionRequest.update({
          where: { id: existingRequest.id },
          data: {
            status: 'PENDING',
            message,
            createdAt: new Date(),
            respondedAt: null
          },
          include: {
            toUser: {
              select: { id: true, name: true, slug: true }
            }
          }
        });

        return res.status(200).json({
          request: updatedRequest,
          message: 'Connection request sent again'
        });
      }
    }

    // Check if reverse request exists (they requested connection to us)
    const reverseRequest = await prisma.connectionRequest.findUnique({
      where: { 
        fromUserId_toUserId: { 
          fromUserId: toUserId, 
          toUserId: fromUserId 
        }
      }
    });

    if (reverseRequest && reverseRequest.status === 'PENDING') {
      // Auto-accept if they already requested connection
      const [connection, , updatedReverse] = await prisma.$transaction([
        // Create connection (ensure consistent ordering)
        prisma.connection.create({
          data: {
            userId1: fromUserId < toUserId ? fromUserId : toUserId,
            userId2: fromUserId < toUserId ? toUserId : fromUserId
          }
        }),
        // Update their request as accepted
        prisma.connectionRequest.update({
          where: { id: reverseRequest.id },
          data: { 
            status: 'ACCEPTED',
            respondedAt: new Date()
          }
        }),
        // Get the updated reverse request with user data
        prisma.connectionRequest.findUnique({
          where: { id: reverseRequest.id },
          include: {
            fromUser: {
              select: { id: true, name: true, slug: true }
            }
          }
        })
      ]);

      return res.status(200).json({
        connection,
        message: 'Connection established! They had already requested to connect with you.'
      });
    }

    // Create new request
    const newRequest = await prisma.connectionRequest.create({
      data: {
        fromUserId,
        toUserId,
        message
      },
      include: {
        toUser: {
          select: { id: true, name: true, slug: true }
        }
      }
    });

    return res.status(201).json({
      request: newRequest,
      message: 'Connection request sent'
    });

  } catch (error) {
    console.error('Connection request error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});