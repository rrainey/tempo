import type { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { serialize } from 'cookie';
import { z } from 'zod';
import { hashPassword } from '../../../lib/auth/hash';
import { issueJWT } from '../../../lib/auth/jwt';

const prisma = new PrismaClient();

const registerSchema = z.object({
  email: z.string().email(),
  password: z.string().min(8),
  name: z.string().min(1),
});

function generateSlug(name: string): string {
  return name
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '');
}

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Validate request body
    const { email, password, name } = registerSchema.parse(req.body);

    // Check if user already exists
    const existingUser = await prisma.user.findUnique({
      where: { email: email.toLowerCase() },
    });

    if (existingUser) {
      return res.status(409).json({ error: 'Email already registered' });
    }

    // Hash password
    const hashedPassword = await hashPassword(password);

    // Generate unique slug
    let slug = generateSlug(name);
    let slugSuffix = 0;
    let finalSlug = slug;

    while (await prisma.user.findUnique({ where: { slug: finalSlug } })) {
      slugSuffix++;
      finalSlug = `${slug}-${slugSuffix}`;
    }

    // Create user
    const user = await prisma.user.create({
      data: {
        email: email.toLowerCase(),
        password: hashedPassword,
        name,
        slug: finalSlug,
        role: 'USER',
      },
    });

    // Issue JWT token (auto-login)
    const token = issueJWT(user);

    // Set cookie
    const cookie = serialize('auth-token', token, {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'lax',
      maxAge: 30 * 24 * 60 * 60, // 30 days
      path: '/',
    });

    res.setHeader('Set-Cookie', cookie);

    // Return user info (without password)
    const { password: _, ...userWithoutPassword } = user;
    return res.status(201).json({
      user: userWithoutPassword,
      success: true,
    });

  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid request data',
        details: error.issues
      });
    }

    console.error('Registration error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
}
