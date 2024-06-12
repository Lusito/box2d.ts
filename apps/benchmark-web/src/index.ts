import { tests, prepareTests, resultsToMarkdown, runTestAsync, TestResult } from "@box2d/benchmark";

import { getBrowserInfo } from "./browserInfo";

const startButton = document.getElementById("start") as HTMLButtonElement;
const tbody = document.querySelector("tbody") as HTMLElement;
const ratioHead = document.getElementById("ratio") as HTMLElement;
const textArea = document.querySelector("textarea") as HTMLTextAreaElement;
const copyButton = document.getElementById("copy") as HTMLButtonElement;

const testRows = prepareTests(tests).map((test) => {
    const row = document.createElement("tr");
    tbody.appendChild(row);
    const name = document.createElement("td");
    const avg = document.createElement("td");
    const p5 = document.createElement("td");
    const p95 = document.createElement("td");
    const ratio = document.createElement("td");
    name.textContent = test.name;
    row.appendChild(name);
    row.appendChild(avg);
    row.appendChild(p5);
    row.appendChild(p95);
    row.appendChild(ratio);
    const progress = document.createElement("progress");
    return {
        row,
        test,
        begin() {
            ratio.appendChild(progress);
            progress.value = 0;
        },
        progress(value: number, max: number) {
            progress.value = value;
            progress.max = max;
        },
        end(result: TestResult) {
            avg.textContent = result.avg.toFixed(2);
            p5.textContent = result.p5.toString();
            p95.textContent = result.p95.toString();
            ratio.textContent = "✓";
        },
        final(ratioValue: number) {
            ratio.textContent = ratioValue.toFixed(2);
            tbody.appendChild(row);
        },
    };
});

type TestResultAndRow = TestResult & { testRow: (typeof testRows)[0] };

export async function runAllTestsAsync() {
    ratioHead.textContent = "";
    const results: TestResultAndRow[] = [];
    for (const testRow of testRows) {
        testRow.begin();
        // eslint-disable-next-line no-await-in-loop
        const result = await runTestAsync(testRow.test, testRow.progress);
        testRow.end(result);
        results.push({ ...result, testRow });
    }

    results.sort((a, b) => a.avg - b.avg).forEach((result) => result.testRow.final(result.avg / results[0].avg));
    startButton.removeAttribute("disabled");
    ratioHead.textContent = "Ratio";
    textArea.value = `**Browser:** ${getBrowserInfo()}\n${resultsToMarkdown(results)}`;
}

startButton.addEventListener("click", () => {
    startButton.disabled = true;
    runAllTestsAsync();
});

copyButton.addEventListener("click", () => {
    textArea.select();
    textArea.setSelectionRange(0, 99999);
    document.execCommand("copy");
    // eslint-disable-next-line no-alert
    alert("Copied to clipboard");
});

if (process.env.NODE_ENV !== "production") {
    new EventSource("/esbuild").addEventListener("change", () => window.location.reload());
}
