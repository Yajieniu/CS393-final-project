#include <vision/structures/Blob.h>

bool sortBlobAreaPredicate(Blob* left, Blob* right) {
  return left->dx * left->dy < right->dx * right->dy;
}

bool BlobCompare(Blob a, Blob b) {
    return a.dx * a.dy > b.dx * b.dy;
}

vector<Blob> filterBlobs(vector<Blob> &blobs, Color color, int size=0) {
    vector<Blob> filtered;
    for(int i = 0; i < blobs.size(); ++i) {
        if(blobs[i].color != color)
            continue;
        if(blobs[i].lpCount < size)
            continue;
        filtered.push_back(blobs[i]);
    }
    sort(filtered.begin(), filtered.end(), BlobCompare);
    return filtered;
}

double calculateBlobAspectRatio(const Blob &b) {
    return (double) b.dx / (double) b.dy;
}

double calculateBlobArea(const Blob &b) {
    return b.dx * b.dy;
}