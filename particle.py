import random


def reverse_sort():
    # reverse sorting
    particles = ['d', 'a', 'b', 'c', 'e']
    orig_prob = [4, 1, 2, 3, 6]

    new_array = particles[:]
    new_prob = orig_prob[:]
    new_prob.sort(reverse=True)
    for index in range(len(new_prob)):
        for inner_index in range(len(new_prob)):
            if orig_prob[index] >= new_prob[inner_index]:
                particles[inner_index] = new_array[index]
                break
    print("particles: ", particles)


# re-sampling wheel algorithm
def re_sample_wheel(iteration=1):
    number_of_samples = 1000
    # weights and probabilities
    w, p = [], []
    p3 = []
    for i in range(iteration):
        for _i in range(number_of_samples):
            # create array for probability
            p.append(random.uniform(0, 500))
            # create array for probability importance weights
            w.append(random.uniform(0, 1.))

        sample_index = random.randint(0, number_of_samples)
        beta = 0

        for _i in range(number_of_samples):
            beta += random.random() * 2 * max(w)
            while w[sample_index] < beta:
                beta -= w[sample_index]
                sample_index = (sample_index + 1) % number_of_samples
                # '% number_of_samples' -> keeps the array reference from going out of bounds
            p3.append(p[sample_index])
    print("p3: ", p3)


reverse_sort()
# re_sample_wheel(2)
