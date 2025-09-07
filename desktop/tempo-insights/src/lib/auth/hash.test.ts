import { hashPassword, verifyPassword } from './hash';

describe('Password hashing', () => {
  it('should hash and verify a password', async () => {
    const password = 'test123';
    const hash = await hashPassword(password);

    // Hash should be different from password
    expect(hash).not.toBe(password);

    // Should verify correctly
    const isValid = await verifyPassword(password, hash);
    expect(isValid).toBe(true);

    // Wrong password should not verify
    const isInvalid = await verifyPassword('wrong', hash);
    expect(isInvalid).toBe(false);
  });

  it('should generate different hashes for same password', async () => {
    const password = 'test123';
    const hash1 = await hashPassword(password);
    const hash2 = await hashPassword(password);

    // Hashes should be different due to salt
    expect(hash1).not.toBe(hash2);

    // Both should verify
    expect(await verifyPassword(password, hash1)).toBe(true);
    expect(await verifyPassword(password, hash2)).toBe(true);
  });
});
