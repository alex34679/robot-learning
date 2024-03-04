from __future__ import division

import numpy as np

try:
    # Python 2 compatibility
    from itertools import izip as zip
except ImportError:
    pass


class Sampler(object):
    def __init__(self, extents):
        """Construct a sampler for the half-open interval defined by extents.

        Args:
            extents: np.array of lower and upper bounds with shape D x 2
        """
        self.extents = extents
        self.dim = extents.shape[0]

    def sample(self, num_samples):
        """Return samples from the sampler.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        raise NotImplementedError


class HaltonSampler(Sampler):
    # This is hard-coded to avoid writing code to generate primes. Halton
    # sequences with larger primes need to drop more initial terms.
    primes = [2, 3, 5, 7, 11, 13]

    def __init__(self, extents):
        super(HaltonSampler, self).__init__(extents)
        self.index = self.primes[self.dim]  # drop the first few terms
        self.gen = self.make_generator()

    def compute_sample(self, index, base):
        """Return an element from the Halton sequence for a desired base.

        This reference may be useful: https://observablehq.com/@jrus/halton.

        Args:
            index: index of the desired element of the Halton sequence
            base: base for the Halton sequence

        Returns:
            the element at index in the base Halton sequence
        """
        # This differs by one from the reference implementation. This excludes 0
        # from our zero-indexed Halton sequence.
        # BEGIN QUESTION 1.1
        index += 1

        fraction = 1
        result = 0

        while(index > 0):
            fraction /= base
            result += fraction * (index % base)
            index = (index // base)

        return result
        # END QUESTION 1.1

    def make_base_generator(self, base):
        """Generate the Halton sequence for a desired base."""
        while True:
            yield self.compute_sample(self.index, base)

    def make_generator(self):
        """Generate the Halton sequence for a list of coprime bases."""
        seqs = [self.make_base_generator(p) for p in self.primes[: self.dim]]
        for x in zip(*seqs):
            yield np.array(x)
            self.index += 1

    def sample(self, num_samples):
        """Return samples from the Halton quasirandom sampler.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        # Generate Halton samples. Each entry lies in the range (0, 1).
        batch = np.empty((num_samples, self.dim))
        for i, x in zip(range(num_samples), self.gen):
            batch[i, :] = x

        # Scale the batch of samples to fit the extents of the space.
        # BEGIN QUESTION 1.1
        #minVal = 0
        #maxVal = num_samples
        difs = abs(self.extents[:, 1] - self.extents[:, 0])
        batch[:] = (batch[:] * difs[:]) + self.extents[:, 0]
        return batch
        # END QUESTION 1.1


class LatticeSampler(Sampler):
    def __init__(self, extents):
        super(LatticeSampler, self).__init__(extents)

    def sample(self, num_samples):
        """Return samples from the lattice sampler.

        Note: this method may return fewer samples than desired.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        volume = np.prod(self.extents[:, 1] - self.extents[:, 0])
        resolution = (volume / num_samples) ** (1.0 / self.dim)
        steps = [
            np.arange(
                self.extents[i, 0] + resolution / 2, self.extents[i, 1], resolution
            )
            for i in range(self.dim)
        ]
        meshed = np.array(np.meshgrid(*steps)).reshape((2, -1)).T
        return meshed[:num_samples, :]


class RandomSampler(Sampler):
    def sample(self, num_samples):
        """Return samples from the random sampler.

        Args:
            num_samples: number of samples to return

        Returns:
            samples: np.array of N x D sample configurations
        """
        return np.random.uniform(
            self.extents[:, 0],
            self.extents[:, 1],
            size=(num_samples, self.dim),
        )


samplers = {
    "halton": HaltonSampler,
    "lattice": LatticeSampler,
    "random": RandomSampler,
}
