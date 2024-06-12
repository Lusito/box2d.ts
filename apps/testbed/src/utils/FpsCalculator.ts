/**
 * Helps calculating a smooth Frames Per Second display
 */
export class FpsCalculator {
    private index = 0;

    private readonly frameTimes: number[];

    private lastTime = -1;

    private readonly waitTime: number;

    private nextUpdate = 0;

    private calculated = {
        avgFps: 0,
        avgTime: 0,
        minTime: 0,
        maxTime: 0,
        minFps: 0,
        maxFps: 0,
    };

    /**
     * @param waitTime Time to wait before updating the time, to avoid flickering
     * @param cacheSize Number of frames to cache
     * @param startValue The start value of all cache items (in milliseconds)
     */
    public constructor(waitTime: number, cacheSize: number, startValue: number) {
        this.waitTime = waitTime;
        this.frameTimes = Array.from({ length: cacheSize }, () => startValue);
        this.calculate();
    }

    public getFps() {
        return this.calculated;
    }

    public getFrames() {
        if (this.index === 0) return this.frameTimes;
        return this.frameTimes.slice(this.index).concat(this.frameTimes.slice(0, this.index));
    }

    public addFrame() {
        const time = performance.now();
        // ignore first frame
        if (this.lastTime === -1) {
            this.lastTime = time;
            return 0;
        }

        const delta = time - this.lastTime;
        this.lastTime = time;

        this.frameTimes[this.index] = delta;
        if (++this.index === this.frameTimes.length) {
            this.index = 0;
        }

        this.nextUpdate -= delta;
        if (this.nextUpdate <= 0) this.calculate();
        return delta;
    }

    private calculate() {
        this.nextUpdate = this.waitTime;
        const sum = this.frameTimes.reduce((a, b) => a + b, 0);

        const avgTime = Math.max(1, sum / this.frameTimes.length);
        const minTime = Math.min(...this.frameTimes);
        const maxTime = Math.max(...this.frameTimes);
        this.calculated = {
            avgFps: 1000 / avgTime,
            avgTime,
            minTime,
            maxTime,
            minFps: 1000 / maxTime,
            maxFps: 1000 / minTime,
        };
    }
}
