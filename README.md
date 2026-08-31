# AXI4 DMA IP

AMBA AXI4 Full 인터페이스의 Burst Transaction을 이용해 데이터 이동을 수행하는 **DMA Controller RTL 설계 프로젝트**입니다.

## Overview

CPU가 직접 반복적인 Memory Copy를 수행하지 않아도 되도록 DMA가 AXI4 Read / Write Transaction을 생성하고, 내부 Buffer를 통해 데이터를 전달하는 구조를 설계합니다.

특히 AXI4의 Burst 특성과 실제 DMA 설계에서 중요한 **4 KB Boundary, Burst Length 분할, 마지막 Burst 처리, Error 처리**를 중점적으로 다룹니다.

## Architecture

```text
                    DMA Controller
        +-----------------------------------+
        |                                   |
        |  +-------------+   +-----------+  |
AXI AR <--- Read Control |   |           |  |
AXI R  ---> Read Path    |-->|   FIFO    |--+--> Write Path --> AXI W
        |  +-------------+   |           |  |                 --> AXI AW
        |                    +-----------+  |                 <-- AXI B
        |                                   |
        +-----------------------------------+
```

## Key Features

- AXI4 Full Read / Write Channel
- INCR Burst 기반 데이터 전송
- Read / Write 독립 제어
- FIFO를 이용한 Read-Write Decoupling
- Burst Length 계산
- 시작 주소가 Burst 경계 중간에 위치하는 경우 처리
- 마지막 Transfer가 최대 Burst 길이에 미달하는 경우 분할
- **4 KB Boundary를 넘지 않도록 Burst 분리**
- 마지막 1-beat Transfer 처리
- AXI Response 기반 Error Detection
- Burst 단위 Error Handling

## AXI4 Channels

### Read

- `ARADDR`
- `ARLEN`
- `ARSIZE`
- `ARBURST`
- `ARVALID / ARREADY`
- `RDATA`
- `RRESP`
- `RLAST`
- `RVALID / RREADY`

### Write

- `AWADDR`
- `AWLEN`
- `AWSIZE`
- `AWBURST`
- `AWVALID / AWREADY`
- `WDATA`
- `WSTRB`
- `WLAST`
- `WVALID / WREADY`
- `BRESP`
- `BVALID / BREADY`

## Burst Calculation

DMA는 요청된 전체 전송 크기를 AXI4 Burst로 나누어 처리합니다.

Burst Length 결정 시 다음 조건을 함께 고려합니다.

```text
Current Address
     ↓
Remaining Transfer Size
     ↓
Max Burst Length
     ↓
4 KB Boundary
     ↓
Next Burst Length
```

핵심 조건:

```text
burst_bytes <= remaining_bytes
burst_bytes <= max_burst_bytes
burst must not cross 4KB boundary
```

## Suggested Module Structure

```text
AXI4_DMA_IP/
├── rtl/
│   ├── dma_top.sv
│   ├── read_engine.sv
│   ├── read_control.sv
│   ├── read_datapath.sv
│   ├── write_engine.sv
│   ├── write_control.sv
│   ├── write_datapath.sv
│   └── fifo.sv
├── tb/
├── docs/
└── README.md
```

> 실제 저장소 파일 구조에 맞게 갱신해 주세요.

## Verification Scenarios

- Aligned Address Transfer
- Unaligned Burst Start Position
- Single Burst Transfer
- Multiple Burst Transfer
- Maximum Burst Length
- Last Short Burst
- Last 1-beat Burst
- 4 KB Boundary Split
- Read Backpressure
- Write Backpressure
- `RRESP` Error
- `BRESP` Error
- Burst-level Error Propagation
- FIFO Full / Empty Condition
- Reset During Idle / Transfer

## Design Focus

이 프로젝트는 단순 AXI Handshake 구현보다 다음 설계 역량을 보여주는 것을 목표로 합니다.

- AXI4 Burst Protocol 이해
- Boundary-aware Address Generation
- Control / Datapath 분리
- FIFO 기반 Pipeline 설계
- Error Policy 정의
- Corner Case 중심 RTL Verification

## Development Environment

- SystemVerilog / Verilog HDL
- AXI4 Full
- RTL Simulation
- Waveform Debugging
