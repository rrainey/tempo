// pages/api/invitations/create.ts
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import crypto from 'crypto';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { proxyUserId } = req.body;
  const creatorId = req.user!.id;

  if (!proxyUserId || typeof proxyUserId !== 'string') {
    return res.status(400).json({ error: 'Proxy user ID is required' });
  }

  try {
    // Verify the proxy user exists and belongs to the creator
    const proxyUser = await prisma.user.findUnique({
      where: { id: proxyUserId },
      include: {
        proxyCreator: true,
      },
    });

    if (!proxyUser) {
      return res.status(404).json({ error: 'Proxy user not found' });
    }

    if (!proxyUser.isProxy) {
      return res.status(400).json({ error: 'User is not a proxy user' });
    }

    if (proxyUser.proxyCreatorId !== creatorId) {
      return res.status(403).json({ error: 'You can only create invitations for your own proxy users' });
    }

    // Check if proxy user already has valid credentials (already claimed)
    if (!proxyUser.email.includes('@tempo.local')) {
      return res.status(400).json({ error: 'This proxy user has already claimed their account' });
    }

    // Check for existing valid invitation
    const existingInvitation = await prisma.userInvitation.findFirst({
      where: {
        email: proxyUser.email,
        expiresAt: { gt: new Date() },
        usedAt: null,
      },
    });

    if (existingInvitation) {
      return res.status(200).json({
        invitation: existingInvitation,
        claimUrl: `${process.env.NEXTAUTH_URL || 'http://localhost:3000'}/accept-invitation/${existingInvitation.code}`,
      });
    }

    // Generate unique invitation code
    const code = crypto.randomBytes(16).toString('hex');
    const expiresAt = new Date();
    expiresAt.setDate(expiresAt.getDate() + 30); // 30 days expiry

    // Create invitation
    const invitation = await prisma.userInvitation.create({
      data: {
        email: proxyUser.email,
        code,
        invitedById: creatorId,
        expiresAt,
        userRole: 'USER', // Proxy users become regular users when claimed
      },
      include: {
        invitedBy: {
          select: { name: true, email: true },
        },
      },
    });

    console.log(`[INVITATION] Created claim invitation for proxy user ${proxyUser.name} (${proxyUser.id})`);

    const claimUrl = `${process.env.NEXTAUTH_URL || 'http://localhost:3000'}/accept-invitation/${code}`;

    return res.status(201).json({
      invitation: {
        id: invitation.id,
        code: invitation.code,
        expiresAt: invitation.expiresAt,
        invitedBy: invitation.invitedBy.name,
      },
      claimUrl,
      proxyUser: {
        id: proxyUser.id,
        name: proxyUser.name,
        slug: proxyUser.slug,
      },
    });
  } catch (error) {
    console.error('Create invitation error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});