// src/pages/api/connections/index.ts
import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method === 'GET') {
    try {
      const userId = req.user!.id;

      // Get all connections
      const connections = await prisma.connection.findMany({
        where: {
          OR: [
            { userId1: userId },
            { userId2: userId }
          ]
        },
        include: {
          user1: {
            select: { id: true, name: true, slug: true, email: true }
          },
          user2: {
            select: { id: true, name: true, slug: true, email: true }
          }
        }
      });

      // Get pending requests (both sent and received)
      const [sentRequests, receivedRequests] = await Promise.all([
        prisma.connectionRequest.findMany({
          where: {
            fromUserId: userId,
            status: 'PENDING'
          },
          include: {
            toUser: {
              select: { id: true, name: true, slug: true, email: true }
            }
          },
          orderBy: { createdAt: 'desc' }
        }),
        prisma.connectionRequest.findMany({
          where: {
            toUserId: userId,
            status: 'PENDING'
          },
          include: {
            fromUser: {
              select: { id: true, name: true, slug: true, email: true }
            }
          },
          orderBy: { createdAt: 'desc' }
        })
      ]);

      // Format connections to always return the other user
      const formattedConnections = connections.map(conn => {
        const otherUser = conn.userId1 === userId ? conn.user2 : conn.user1;
        return {
          id: conn.id,
          user: otherUser,
          createdAt: conn.createdAt
        };
      });

      return res.status(200).json({
        connections: formattedConnections,
        pendingRequests: {
          sent: sentRequests,
          received: receivedRequests
        }
      });

    } catch (error) {
      console.error('List connections error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  } else if (req.method === 'DELETE') {
    // Remove a connection
    try {
      const { connectionId } = req.body;
      
      if (!connectionId || typeof connectionId !== 'string') {
        return res.status(400).json({ error: 'Invalid connection ID' });
      }

      const userId = req.user!.id;

      // Find the connection
      const connection = await prisma.connection.findUnique({
        where: { id: connectionId }
      });

      if (!connection) {
        return res.status(404).json({ error: 'Connection not found' });
      }

      // Verify user is part of this connection
      if (connection.userId1 !== userId && connection.userId2 !== userId) {
        return res.status(403).json({ error: 'You cannot remove this connection' });
      }

      // Delete the connection
      await prisma.connection.delete({
        where: { id: connectionId }
      });

      return res.status(200).json({ message: 'Connection removed' });

    } catch (error) {
      console.error('Remove connection error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  } else {
    return res.status(405).json({ error: 'Method not allowed' });
  }
});