// place files you want to import through the `$lib` alias in this folder.

export const width = 144;
export const height = 144;

export const distance_sensor_max = 2000 * 0.0393701;

export const use_normal_dist = false;

// https://stackoverflow.com/questions/25582882/javascript-math-random-normal-distribution-gaussian-bell-curve
export function gaussian_random(mean: number, stdev: number) {
    const u = 1 - Math.random(); // Converting [0,1) to (0,1]
    const v = Math.random();
    const z = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    // Transform to the desired mean and standard deviation:
    return z * stdev + mean;
}