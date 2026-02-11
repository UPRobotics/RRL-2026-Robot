#pragma once

#include <SDL2/SDL.h>
#include <vector>
#include <array>
#include <functional>
#include "camera_pkg/types.h"

namespace camera_viewer {

enum class ViewMode;
class CameraManager;

/**
 * @brief Camera grid component managing camera layout and rendering
 */
class CameraGrid {
public:
    CameraGrid();
    ~CameraGrid() = default;

    void setCameraManager(CameraManager* manager);
    void setAvailableCameraIndices(const std::vector<int>& indices);
    void setViewMode(ViewMode mode);

    int getSelectedCameraIndex() const { return m_selectedCameraIndex; }
    int getSelectedRealCameraIndex() const { return m_selectedRealCameraIndex; }
    int getAvailableCameraCount() const { return static_cast<int>(m_availableCameraIndices.size()); }

    void setSelectedCameraIndex(int index);

    void set2x2SlotAssignments(const std::array<int, 4>& slots);
    const std::array<int, 4>& get2x2SlotAssignments() const { return m_2x2Slots; }
    void cycle2x2Slot(int slotIndex);

    void render(SDL_Renderer* renderer, int x, int y, int width, int height);

private:
    void calculateGridLayout(int& rows, int& cols);
    void renderCameraSlot(SDL_Renderer* renderer, SDL_Rect rect, int cameraIndex);
    void renderVideoTexture(SDL_Renderer* renderer, SDL_Rect rect, SDL_Texture* texture, int cameraIndex);
    void renderPlaceholder(SDL_Renderer* renderer, SDL_Rect rect, int cameraIndex);
    SDL_Rect calculateFitRect(SDL_Rect target, int srcWidth, int srcHeight);

    CameraManager* m_cameraManager = nullptr;
    std::vector<int> m_availableCameraIndices;
    ViewMode m_viewMode;
    int m_selectedCameraIndex;
    int m_selectedRealCameraIndex;

    std::array<int, 4> m_2x2Slots = {-1, -1, -1, -1};

    static constexpr int CAMERA_GAP = 1;
    static constexpr int CAMERA_BORDER = 1;
};

} // namespace camera_viewer
