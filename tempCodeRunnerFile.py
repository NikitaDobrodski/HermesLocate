    try:
            initial_x, initial_y = self.initial_coordinates
            self.support_positioning.setup(self.frame_queue.get(), initial_x, initial_y)

            while not self.stop_event.is_set():
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    self.processed_frames += 1

                    frame_start_time = time.perf_counter()

                    if self.processed_frames % COORDINATE_INTERVAL == 0:
                        # Получение координат из обработки видео
                        coordinate = self.support_positioning.calculate_coordinate(frame, initial_x, initial_y)
                        
                        # Получение других параметров из полетного контроллера
                        altitude = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).relative_alt / 1000.0
                        azimuth = self.master.recv_match(type='VFR_HUD', blocking=True).heading
                        satellites = 30  # Фиксированное количество спутников для подмены

                        # Преобразование координат в формат для MAVLink
                        lat = int(coordinate[0] * 1e7)
                        lon = int(coordinate[1] * 1e7)

                        # Ограничение значений для формата int32
                        lat = max(-2147483648, min(2147483647, lat))
                        lon = max(-2147483648, min(2147483647, lon))

                        # Отправка данных на полетный контроллер
                        self.master.mav.gps_input_send(
                            int(time.time() * 1e6),
                            0,
                            0,
                            0,
                            0,
                            3,
                            lat,
                            lon,
                            altitude,
                            float(0.7),
                            float(1.0),
                            0,
                            0,
                            0,
                            float(0.1),
                            float(0.5),
                            float(0.5),
                            satellites,
                            0
                        )

                    frame_end_time = time.perf_counter()
                    processing_time = frame_end_time - frame_start_time

                    time.sleep(1 / FPS)
        except Exception as e:
            print(f"Error processing frames: {e}")
        finally:
            self.stop_event.set()