import cv2
import time
import random
from threading import Semaphore, Thread

# Semáforo para controlar o acesso à área crítica
semaphore = Semaphore(1)


# Função para processar um segmento do vídeo em escala de cinza
def process_video_segment(frames):
    output_frames = []
    for frame in frames:
        if frame is not None:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            output_frames.append(gray_frame)
        else:
            output_frames.append(None)
    return output_frames


# Função para dividir o vídeo em segmentos com tamanhos aleatórios
def load_video_segments(video_path, num_segments=4):
    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()

    # Gerar tamanhos de segmento aleatórios
    segment_sizes = [random.randint(1, total_frames // num_segments) for _ in range(num_segments)]

    while sum(segment_sizes) < total_frames:
        segment_sizes[random.randint(0, num_segments - 1)] += 1
    segment_sizes[-1] += total_frames - sum(segment_sizes)

    video_segments = []
    start = 0
    for size in segment_sizes:
        end = start + size
        video_segments.append(frames[start:end])
        start = end
        if start >= total_frames:
            break

    return video_segments, width, height, fps


# Algoritmo FIFO
def fifo(video_segments):
    output_frames = []
    wait_times = [0] * len(video_segments)
    total_processing_time = 0

    for segment_id, frames in enumerate(video_segments):
        wait_times[segment_id] = total_processing_time / 1000
        output_frames.extend(process_video_segment(frames))
        total_processing_time += len(frames)

    return output_frames, wait_times


# Algoritmo Round Robin
def round_robin(video_segments, quantum=20):
    output_frames = []
    segment_indices = [0] * len(video_segments)
    wait_times = [0] * len(video_segments)
    time_elapsed = 0

    while True:
        all_done = True
        for segment_id, frames in enumerate(video_segments):
            start_idx = segment_indices[segment_id]
            if start_idx < len(frames):
                all_done = False
                end_idx = min(start_idx + quantum, len(frames))
                wait_times[segment_id] += (time_elapsed / 1000) - wait_times[segment_id]
                output_frames.extend(process_video_segment(frames[start_idx:end_idx]))
                segment_indices[segment_id] += (end_idx - start_idx)
                time_elapsed += (end_idx - start_idx)

        for segment_id, frames in enumerate(video_segments):
            if segment_indices[segment_id] < len(frames):
                wait_times[segment_id] += quantum

        if all_done:
            break

    return output_frames, wait_times


# Algoritmo Shortest Job First (SJF)
def shortest_job_first(video_segments):
    sorted_segments = sorted(enumerate(video_segments), key=lambda x: len(x[1]))

    wait_times = [0] * len(video_segments)
    output_frames = []
    total_processing_time = 0

    for i, (segment_id, frames) in enumerate(sorted_segments):
        if i > 0:
            wait_times[segment_id] = total_processing_time / 1000
        output_frames.extend(process_video_segment(frames))
        # Atualiza o tempo total de processamento
        total_processing_time += len(frames)

    return output_frames, wait_times


# Função para salvar o vídeo processado
def save_video(output_frames, width, height, fps, filename):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, fps, (width, height), isColor=False)

    for gray_frame in output_frames:
        if gray_frame is not None:
            out.write(gray_frame)
        else:
            print("Frame vazio encontrado e ignorado.")

    out.release()


# Função que encapsula o processamento de cada algoritmo
def process_algorithm(algorithm_name, video_segments, width, height, fps, execution_times,
                      execution_number, num_executions, waiting_times):
    print(f"{algorithm_name} está tentando acessar a seção crítica...")
    with semaphore:  # Acesso à área crítica
        print(f"{algorithm_name} obteve acesso à seção crítica.")
        algorithm_start_time = time.time()  # Tempo dentro da seção crítica
        if algorithm_name == "FIFO":
            output, waiting_times_per_algorithm = fifo(video_segments)
            if execution_number + 1 == num_executions:
                save_video(output, width, height, fps, "output_fifo.mp4")
        elif algorithm_name == "SJF":
            output, waiting_times_per_algorithm = shortest_job_first(video_segments)
            if execution_number + 1 == num_executions:
                save_video(output, width, height, fps, "output_sjf.mp4")
        elif algorithm_name == "Round Robin":
            output, waiting_times_per_algorithm = round_robin(video_segments)
            if execution_number + 1 == num_executions:
                save_video(output, width, height, fps, "output_round_robin.mp4")
        exec_time = time.time() - algorithm_start_time
        execution_times[algorithm_name].append(exec_time)
        waiting_times[algorithm_name].append((sum(waiting_times_per_algorithm)) / 4)
        print(f"{algorithm_name} finalizou o acesso a seção crítica...")
        print(f"{algorithm_name} tempo de espera médio: {((sum(waiting_times_per_algorithm)) / 4):.4f} segundos...")


# Função principal para executar e medir cada algoritmo
def main():
    video_path = "video.mp4"

    num_executions = 10
    execution_times = {
        "FIFO": [],
        "Round Robin": [],
        "SJF": []
    }

    waiting_times = {
        "FIFO": [],
        "Round Robin": [],
        "SJF": []
    }

    for execution_number in range(num_executions):
        video_segments, width, height, fps = load_video_segments(video_path)

        threads = []
        threads.append(
            Thread(target=process_algorithm, args=("FIFO", video_segments, width, height, fps,
                                                   execution_times, execution_number, num_executions, waiting_times)))
        threads.append(
            Thread(target=process_algorithm, args=("SJF", video_segments, width, height, fps,
                                                   execution_times, execution_number, num_executions, waiting_times)))
        threads.append(
            Thread(target=process_algorithm, args=("Round Robin", video_segments, width, height, fps,
                                                   execution_times, execution_number, num_executions, waiting_times)))

        random.shuffle(threads)

        # Inicia as threads
        for thread in threads:
            thread.start()

        # Aguarda todas as threads terminarem
        for thread in threads:
            thread.join()

        print(f"Execução [{execution_number + 1}] finalizada. \n\n")

    print(f"Média de tempo de execução FIFO: {sum(execution_times['FIFO']) / num_executions:.4f} segundos")
    print(f"Média de tempo de espera FIFO: {sum(waiting_times['FIFO']) / num_executions:.4f} segundos")

    print(
        f"Média de tempo de execução Round Robin: {sum(execution_times['Round Robin']) / num_executions:.4f} segundos")
    print(f"Média de tempo de espera Round Robin: {sum(waiting_times['Round Robin']) / num_executions:.4f} segundos")

    print(f"Média de tempo de execução Shortest Job First: {sum(execution_times['SJF']) / num_executions:.4f} segundos")
    print(f"Média de tempo de espera Shortest Job First: {sum(waiting_times['SJF']) / num_executions:.4f} segundos")


# Resultado após 10 execuções.
# O Roundo Robin deu um valor estranho, provavelmente implementei errado a lógica dele.
# Média de tempo de execução FIFO: 0.7200 segundos
# Média de tempo de espera FIFO: 0.5441 segundos
# Média de tempo de execução Round Robin: 0.7776 segundos
# Média de tempo de espera Round Robin: 1.2597 segundos
# Média de tempo de execução Shortest Job First: 0.6956 segundos
# Média de tempo de espera Shortest Job First: 0.4510 segundos

if __name__ == "__main__":
    main()
