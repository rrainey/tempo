// src/pages/api/connections/[id]/respond.ts
import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const respondSchema = z.object({
  action: z.enum(['accept', 'decline'])
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { id: requestId } = req.query;
    
    if (!requestId || typeof requestId !== 'string') {
      return res.status(400).json({ error: 'Invalid request ID' });
    }

    const validation = respondSchema.safeParse(req.body);
    if (!validation.success) {
      return res.status(400).json({ error: 'Invalid input', details: validation.error });
    }

    const { action } = validation.data;
    const userId = req.user!.id;

    // Get the connection request
    const request = await prisma.connectionRequest.findUnique({
      where: { id: requestId },
      include: {
        fromUser: true,
        toUser: true
      }
    });

    if (!request) {
      return res.status(404).json({ error: 'Connection request not found' });
    }

    // Verify the current user is the recipient
    if (request.toUserId !== userId) {
      return res.status(403).json({ error: 'You cannot respond to this request' });
    }

    // Check if already responded
    if (request.status !== 'PENDING') {
      return res.status(400).json({ 
        error: `Request already ${request.status.toLowerCase()}` 
      });
    }

    if (action === 'accept') {
      // Accept the request
      const [connection, updatedRequest] = await prisma.$transaction([
        // Create connection (ensure consistent ordering)
        prisma.connection.create({
          data: {
            userId1: request.fromUserId < request.toUserId ? request.fromUserId : request.toUserId,
            userId2: request.fromUserId < request.toUserId ? request.toUserId : request.fromUserId
          }
        }),
        // Update request status
        prisma.connectionRequest.update({
          where: { id: requestId },
          data: {
            status: 'ACCEPTED',
            respondedAt: new Date()
          }
        })
      ]);

      return res.status(200).json({
        connection,
        message: `Connected with ${request.fromUser.name}`
      });
    } else {
      // Decline the request
      const updatedRequest = await prisma.connectionRequest.update({
        where: { id: requestId },
        data: {
          status: 'DECLINED',
          respondedAt: new Date()
        }
      });

      return res.status(200).json({
        request: updatedRequest,
        message: 'Connection request declined'
      });
    }

  } catch (error) {
    console.error('Connection respond error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});