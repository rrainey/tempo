// src/pages/review/fs/[id].tsx
import { useRouter } from 'next/router';
import { AppLayout } from '../../../components/AppLayout';
import { AuthGuard } from '../../../components/AuthGuard';
import { FormationReview } from '../../../components/formation/FormationReview';

export default function FormationReviewPage() {
  const router = useRouter();
  const { id } = router.query;

  if (!id || typeof id !== 'string') {
    return (
      <AuthGuard>
        <AppLayout>
          <div>Loading...</div>
        </AppLayout>
      </AuthGuard>
    );
  }

  return (
    <AuthGuard>
      <AppLayout>
        <FormationReview formationId={id} />
      </AppLayout>
    </AuthGuard>
  );
}