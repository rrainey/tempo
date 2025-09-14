// lib/auth/verify-ssr.ts

import { GetServerSidePropsContext } from 'next';
import { parse } from 'cookie';
import { verifyJWT } from './jwt';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export interface AuthUser {
  id: string;
  email: string;
  name: string;
  role: string;
}

export async function verifyAuthSSR(context: GetServerSidePropsContext): Promise<AuthUser | null> {
  try {
    // Parse cookies from request
    const cookies = parse(context.req.headers.cookie || '');
    const token = cookies['auth-token'];
    
    if (!token) {
      return null;
    }

    // Verify JWT
    const payload = verifyJWT(token);
    
    // Get user from database
    const user = await prisma.user.findUnique({
      where: { id: payload.userId },
      select: {
        id: true,
        email: true,
        name: true,
        role: true,
      },
    });

    if (!user) {
      return null;
    }

    return user;
  } catch (error) {
    console.error('SSR auth verification error:', error);
    return null;
  }
}