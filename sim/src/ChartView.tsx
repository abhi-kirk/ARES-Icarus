import { useEffect, useRef } from "react";
import * as echarts from "echarts";
import type { EChartsOption } from "echarts";

export function ChartView({ option, layoutKey }: { option: EChartsOption; layoutKey?: number }) {
  const host = useRef<HTMLDivElement>(null);
  const chart = useRef<echarts.ECharts | null>(null);

  useEffect(() => {
    if (!host.current) return;
    const instance = echarts.init(host.current);
    chart.current = instance;
    const resize = () => instance.resize();
    const observer = new ResizeObserver(resize);
    observer.observe(host.current);
    const parent = host.current.parentElement;
    if (parent) observer.observe(parent);
    return () => {
      observer.disconnect();
      instance.dispose();
      chart.current = null;
    };
  }, []);

  useEffect(() => {
    chart.current?.setOption(option, true);
  }, [option]);

  useEffect(() => {
    if (layoutKey === undefined) return;
    chart.current?.resize();
  }, [layoutKey]);

  return <div className="chart" ref={host} />;
}
