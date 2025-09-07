import { issueJWT, verifyJWT } from './jwt';
import { User, UserRole } from '@prisma/client';
import jwt from 'jsonwebtoken';

// Set JWT_SECRET for tests if not already set
process.env.JWT_SECRET = process.env.JWT_SECRET || 'test-secret-key-for-testing-only';

// Mock user for testing
const mockUser: User = {
  id: 'test-user-id',
  email: 'test@example.com',
  password: 'hashed-password',
  name: 'Test User',
  slug: 'test-user',
  role: UserRole.USER,
  createdAt: new Date(),
  updatedAt: new Date(),
};

describe('JWT utilities', () => {
  it('should issue and verify a JWT token', () => {
    const token = issueJWT(mockUser);

    // Token should be a string
    expect(typeof token).toBe('string');
    expect(token.split('.')).toHaveLength(3); // JWT has 3 parts

    // Verify the token
    const decoded = verifyJWT(token);
    expect(decoded.userId).toBe(mockUser.id);
    expect(decoded.email).toBe(mockUser.email);
    expect(decoded.role).toBe(mockUser.role);
  });

  it('should throw error on invalid token', () => {
    expect(() => {
      verifyJWT('invalid-token');
    }).toThrow();
  });

  it('should include expiration in token', () => {
    const token = issueJWT(mockUser);
    const decoded = jwt.decode(token) as any;

    expect(decoded.exp).toBeDefined();

    // Check that expiration is roughly 30 days from now
    const expirationDate = new Date(decoded.exp * 1000);
    const expectedExpiration = new Date();
    expectedExpiration.setDate(expectedExpiration.getDate() + 30);

    // Allow 1 minute tolerance for test execution time
    const diff = Math.abs(expirationDate.getTime() - expectedExpiration.getTime());
    expect(diff).toBeLessThan(60000); // Less than 1 minute difference
  });
});
