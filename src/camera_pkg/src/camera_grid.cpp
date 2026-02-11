#include "camera_pkg/camera_grid.h"
#include "camera_pkg/camera_manager.h"
#include "camera_pkg/main_window.h"
#include "camera_pkg/ui_helpers.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

namespace camera_viewer {

CameraGrid::CameraGrid()
    : m_viewMode(ViewMode::GRID_NXN)
    , m_selectedCameraIndex(0)
    , m_selectedRealCameraIndex(-1)
{
}

void CameraGrid::setCameraManager(CameraManager* manager) {
    m_cameraManager = manager;
}

void CameraGrid::setAvailableCameraIndices(const std::vector<int>& indices) {
    m_availableCameraIndices = indices;
    
    if (m_availableCameraIndices.empty()) {
        m_selectedCameraIndex = 0;
        m_selectedRealCameraIndex = -1;
    } else {
        if (m_selectedCameraIndex >= static_cast<int>(m_availableCameraIndices.size())) {
            m_selectedCameraIndex = 0;
        }
        m_selectedRealCameraIndex = m_availableCameraIndices[m_selectedCameraIndex];
    }
}

void CameraGrid::setViewMode(ViewMode mode) {
    m_viewMode = mode;
}

void CameraGrid::setSelectedCameraIndex(int index) {
    if (index >= 0 && index < static_cast<int>(m_availableCameraIndices.size())) {
        m_selectedCameraIndex = index;
        m_selectedRealCameraIndex = m_availableCameraIndices[index];
    }
}

void CameraGrid::set2x2SlotAssignments(const std::array<int, 4>& slots) {
    m_2x2Slots = slots;
}

void CameraGrid::cycle2x2Slot(int slotIndex) {
    if (slotIndex < 0 || slotIndex >= 4) return;
    if (m_availableCameraIndices.empty()) return;

    int currentCam = m_2x2Slots[slotIndex];

    int currentPos = -1;
    for (int i = 0; i < static_cast<int>(m_availableCameraIndices.size()); ++i) {
        if (m_availableCameraIndices[i] == currentCam) {
            currentPos = i;
            break;
        }
    }

    int nextPos = (currentPos + 1) % static_cast<int>(m_availableCameraIndices.size());
    m_2x2Slots[slotIndex] = m_availableCameraIndices[nextPos];
}

void CameraGrid::render(SDL_Renderer* renderer, int x, int y, int width, int height) {
    int activeCameraCount = static_cast<int>(m_availableCameraIndices.size());
    
    if (activeCameraCount == 0) {
        UIHelpers::drawText(renderer, width / 2 - 60, height / 2, 
                           "No cameras active", Colors::CAMERA_TEXT, 14);
        return;
    }

    if (m_viewMode == ViewMode::FULLSCREEN) {
        SDL_Rect cameraRect = {x + CAMERA_GAP, y + CAMERA_GAP, 
                              width - CAMERA_GAP * 2, height - CAMERA_GAP * 2};
        renderCameraSlot(renderer, cameraRect, m_selectedRealCameraIndex);
    } else if (m_viewMode == ViewMode::GRID_2X2) {
        int rows = 2, cols = 2;

        int totalGapX = CAMERA_GAP * (cols + 1);
        int totalGapY = CAMERA_GAP * (rows + 1);
        int baseWidth = (width - totalGapX) / cols;
        int baseHeight = (height - totalGapY) / rows;
        int extraWidth = (width - totalGapX) % cols;
        int extraHeight = (height - totalGapY) % rows;

        std::vector<int> colWidths(cols, baseWidth);
        for (int i = 0; i < extraWidth; ++i) colWidths[i] += 1;
        std::vector<int> rowHeights(rows, baseHeight);
        for (int i = 0; i < extraHeight; ++i) rowHeights[i] += 1;

        int cameraForSlot[4];
        for (int s = 0; s < 4; ++s) {
            if (m_2x2Slots[s] >= 0) {
                cameraForSlot[s] = m_2x2Slots[s];
            } else if (s < activeCameraCount) {
                cameraForSlot[s] = m_availableCameraIndices[s];
            } else {
                cameraForSlot[s] = -1;
            }
        }

        int slotIdx = 0;
        int cameraY = y + CAMERA_GAP;
        for (int row = 0; row < rows; ++row) {
            int cameraX = x + CAMERA_GAP;
            for (int col = 0; col < cols; ++col) {
                SDL_Rect cameraRect = {cameraX, cameraY, colWidths[col], rowHeights[row]};
                int realCameraIndex = cameraForSlot[slotIdx];
                if (realCameraIndex >= 0) {
                    renderCameraSlot(renderer, cameraRect, realCameraIndex);
                }
                slotIdx++;
                cameraX += colWidths[col] + CAMERA_GAP;
            }
            cameraY += rowHeights[row] + CAMERA_GAP;
        }
    } else {
        int rows, cols;
        calculateGridLayout(rows, cols);

        int totalGapX = CAMERA_GAP * (cols + 1);
        int totalGapY = CAMERA_GAP * (rows + 1);
        int baseWidth = (width - totalGapX) / cols;
        int baseHeight = (height - totalGapY) / rows;
        int extraWidth = (width - totalGapX) % cols;
        int extraHeight = (height - totalGapY) % rows;

        std::vector<int> colWidths(cols, baseWidth);
        for (int i = 0; i < extraWidth; ++i) colWidths[i] += 1;

        std::vector<int> rowHeights(rows, baseHeight);
        for (int i = 0; i < extraHeight; ++i) rowHeights[i] += 1;

        int slotIndex = 0;
        int cameraY = y + CAMERA_GAP;
        for (int row = 0; row < rows && slotIndex < activeCameraCount; ++row) {
            int cameraX = x + CAMERA_GAP;
            for (int col = 0; col < cols && slotIndex < activeCameraCount; ++col) {
                int cameraWidth = colWidths[col];
                int cameraHeight = rowHeights[row];

                SDL_Rect cameraRect = {cameraX, cameraY, cameraWidth, cameraHeight};
                int realCameraIndex = m_availableCameraIndices[slotIndex];
                renderCameraSlot(renderer, cameraRect, realCameraIndex);
                slotIndex++;
                cameraX += cameraWidth + CAMERA_GAP;
            }
            cameraY += rowHeights[row] + CAMERA_GAP;
        }
    }
}

void CameraGrid::calculateGridLayout(int& rows, int& cols) {
    int count = static_cast<int>(m_availableCameraIndices.size());
    if (count <= 1) {
        rows = 1; cols = 1;
    } else if (count <= 4) {
        rows = 2; cols = 2;
    } else if (count <= 9) {
        rows = 3; cols = 3;
    } else if (count <= 16) {
        rows = 4; cols = 4;
    } else {
        cols = static_cast<int>(std::ceil(std::sqrt(count)));
        rows = static_cast<int>(std::ceil(static_cast<float>(count) / cols));
    }
}

void CameraGrid::renderCameraSlot(SDL_Renderer* renderer, SDL_Rect rect, int cameraIndex) {
    SDL_SetRenderDrawColor(renderer, 
        Colors::CAMERA_BORDER.r, Colors::CAMERA_BORDER.g, 
        Colors::CAMERA_BORDER.b, Colors::CAMERA_BORDER.a);
    for (int i = 0; i < CAMERA_BORDER; ++i) {
        SDL_Rect borderRect = {rect.x - i, rect.y - i, rect.w + i * 2, rect.h + i * 2};
        SDL_RenderDrawRect(renderer, &borderRect);
    }

    SDL_SetRenderDrawColor(renderer, 
        Colors::CAMERA_BG.r, Colors::CAMERA_BG.g, 
        Colors::CAMERA_BG.b, Colors::CAMERA_BG.a);
    SDL_RenderFillRect(renderer, &rect);

    CameraStats stats;
    SDL_Texture* texture = nullptr;
    
    if (m_cameraManager) {
        texture = m_cameraManager->getCameraTexture(cameraIndex);
        stats = m_cameraManager->getCameraStats(cameraIndex);
    }

    if (texture && stats.state == CameraState::Connected) {
        renderVideoTexture(renderer, rect, texture, cameraIndex);
    } else {
        renderPlaceholder(renderer, rect, cameraIndex);
    }

    if (cameraIndex == m_selectedRealCameraIndex) {
        SDL_SetRenderDrawColor(renderer, 
            Colors::BUTTON_HOVER.r, Colors::BUTTON_HOVER.g, 
            Colors::BUTTON_HOVER.b, 255);
        for (int i = 0; i < 3; ++i) {
            SDL_Rect highlightRect = {
                rect.x - CAMERA_BORDER - i, 
                rect.y - CAMERA_BORDER - i, 
                rect.w + (CAMERA_BORDER + i) * 2, 
                rect.h + (CAMERA_BORDER + i) * 2
            };
            SDL_RenderDrawRect(renderer, &highlightRect);
        }
    }
}

SDL_Rect CameraGrid::calculateFitRect(SDL_Rect target, int srcWidth, int srcHeight) {
    if (srcWidth <= 0 || srcHeight <= 0) return target;

    float targetAspect = static_cast<float>(target.w) / target.h;
    float srcAspect = static_cast<float>(srcWidth) / srcHeight;

    SDL_Rect result;
    if (srcAspect > targetAspect) {
        result.w = target.w;
        result.h = static_cast<int>(target.w / srcAspect);
        result.x = target.x;
        result.y = target.y + (target.h - result.h) / 2;
    } else {
        result.h = target.h;
        result.w = static_cast<int>(target.h * srcAspect);
        result.x = target.x + (target.w - result.w) / 2;
        result.y = target.y;
    }
    return result;
}

void CameraGrid::renderVideoTexture(SDL_Renderer* renderer, SDL_Rect rect, SDL_Texture* texture, int cameraIndex) {
    int rotation = 0;
    if (m_cameraManager) {
        rotation = m_cameraManager->getCameraRotation(cameraIndex);
    }

    int texW, texH;
    SDL_QueryTexture(texture, nullptr, nullptr, &texW, &texH);

    int fitW = texW;
    int fitH = texH;
    if (rotation % 180 != 0) {
        fitW = texH;
        fitH = texW;
    }
    
    SDL_Rect destRect = calculateFitRect(rect, fitW, fitH);
    
    if (destRect.x > rect.x || destRect.y > rect.y) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderFillRect(renderer, &rect);
    }
    
    SDL_RenderCopyEx(renderer, texture, nullptr, &destRect, static_cast<double>(rotation), nullptr, SDL_FLIP_NONE);
}

void CameraGrid::renderPlaceholder(SDL_Renderer* renderer, SDL_Rect rect, int cameraIndex) {
    (void)cameraIndex;
    
    SDL_SetRenderDrawColor(renderer, 60, 60, 65, 255);
    int gridSize = 40;
    for (int i = gridSize; i < rect.w; i += gridSize) {
        SDL_RenderDrawLine(renderer, rect.x + i, rect.y, rect.x + i, rect.y + rect.h);
    }
    for (int i = gridSize; i < rect.h; i += gridSize) {
        SDL_RenderDrawLine(renderer, rect.x, rect.y + i, rect.x + rect.w, rect.y + i);
    }
}

} // namespace camera_viewer
