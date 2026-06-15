# LibXR local commit backup

This directory preserves the local-only LibXR commit used by this project:

- commit: `16356ec1e2c0ec13546d73ac487b460f7c1698ec`
- subject: local optimization commit
- local branch: `recover/iamcmm11-libxr-20260604`
- local tag: `preserve/libxr-local-16356ec`

Files:

- `0001-libxr-local-16356ec.patch`: portable patch for `git am`
- `libxr-local-16356ec.bundle`: Git bundle containing the commit object

If `Middlewares/Third_Party/LibXR` is reset to the remote version, restore it
with:

```sh
cd Middlewares/Third_Party/LibXR
git fetch ../../../patches/libxr/libxr-local-16356ec.bundle refs/tags/preserve/libxr-local-16356ec:refs/heads/recover/iamcmm11-libxr-20260604
git checkout recover/iamcmm11-libxr-20260604
```

Alternatively, apply the patch to the matching LibXR base:

```sh
cd Middlewares/Third_Party/LibXR
git am ../../../patches/libxr/0001-libxr-local-16356ec.patch
```
