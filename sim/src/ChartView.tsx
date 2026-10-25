import { useEffect, useRef } from "react";
import * as echarts from "echarts";
import type { EChartsOption } from "echarts";

export function ChartView({
  option,
  layoutKey,
  sceneKey,
  onScrub,
}: {
  option: EChartsOption;
  layoutKey?: number;
  sceneKey?: string;
  onScrub?: (time: number) => void;
}) {
  const host = useRef<HTMLDivElement>(null);
  const chart = useRef<echarts.ECharts | null>(null);
  const scrubRef = useRef(onScrub);
  scrubRef.current = onScrub;

  useEffect(() => {
    if (!host.current) return;
    const instance = echarts.init(host.current);
    chart.current = instance;
    const resize = () => instance.resize();
    const observer = new ResizeObserver(resize);
    observer.observe(host.current);
    const parent = host.current.parentElement;
    if (parent) observer.observe(parent);

    const zr = instance.getZr();
    let dragging = false;

    const scrubAt = (offsetX: number, offsetY: number) => {
      const scrub = scrubRef.current;
      if (!scrub) return;
      if (!instance.containPixel({ gridIndex: 0 }, [offsetX, offsetY])) return;
      const value = instance.convertFromPixel({ xAxisIndex: 0 }, [offsetX, offsetY]);
      const time = Array.isArray(value) ? value[0] : value;
      if (typeof time === "number" && Number.isFinite(time)) scrub(time);
    };

    const down = (event: { offsetX: number; offsetY: number }) => {
      dragging = true;
      scrubAt(event.offsetX, event.offsetY);
    };
    const move = (event: { offsetX: number; offsetY: number }) => {
      if (!dragging) return;
      scrubAt(event.offsetX, event.offsetY);
    };
    const up = () => {
      dragging = false;
    };

    zr.on("mousedown", down);
    zr.on("mousemove", move);
    zr.on("mouseup", up);
    zr.on("globalout", up);

    return () => {
      zr.off("mousedown", down);
      zr.off("mousemove", move);
      zr.off("mouseup", up);
      zr.off("globalout", up);
      observer.disconnect();
      instance.dispose();
      chart.current = null;
    };
  }, []);

  useEffect(() => {
    chart.current?.setOption(option, { replaceMerge: ["yAxis", "series", "tooltip"], lazyUpdate: true });
  }, [option]);

  useEffect(() => {
    if (!sceneKey || !chart.current) return;
    chart.current.setOption(
      { animation: true, animationDuration: 220, animationEasing: "cubicOut" },
      { notMerge: false, lazyUpdate: true },
    );
  }, [sceneKey]);

  useEffect(() => {
    if (layoutKey === undefined) return;
    chart.current?.resize();
  }, [layoutKey]);

  return <div className={onScrub ? "chart is-scrub" : "chart"} ref={host} />;
}
